#!/usr/bin/env python3
"""
Motorized Finder - ESP32 Motor Controller - Pygame Interface
Controls focus and zoom motors via COBS-framed binary commands.

Controls:
  RIGHT Arrow    Focus Forward
  LEFT Arrow     Focus Backward
  UP Arrow       Zoom Forward
  DOWN Arrow     Zoom Backward
  [ / ]          Decrease / Increase focus speed (step 10)
  ; / '          Decrease / Increase zoom speed (step 10)
  P              Toggle pulse mode (timed) / continuous mode
  - / =          Decrease / Increase pulse duration (step 10ms, pulse mode only)
  T              Run focus sweep test (min→max at slow/medium/fast speed)
  Y              Run zoom sweep test
  ESC            Exit / cancel test

  In pulse mode each arrow keypress fires a single timed command (default 150ms);
  the motor self-stops after the duration — no key release required.
"""

import json
import os
import pygame
import serial
import serial.tools.list_ports
import sys
import time
import struct
from datetime import datetime

# ── Serial configuration ──────────────────────────────────────────────────────

BAUD_RATE = 115200

# ── Colors ────────────────────────────────────────────────────────────────────

BLACK      = (0,   0,   0)
WHITE      = (255, 255, 255)
GREEN      = (0,   220, 80)
RED        = (220, 50,  50)
BLUE       = (60,  140, 255)
GRAY       = (80,  80,  80)
DARK_GRAY  = (40,  40,  40)
YELLOW     = (240, 200, 0)
ORANGE     = (255, 140, 0)
PANEL_BG   = (18,  18,  30)

# ── Protocol helpers ──────────────────────────────────────────────────────────

def crc8(data: bytes) -> int:
    """CRC8 Dallas/Maxim, polynomial 0x31."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out      = bytearray()
    code_idx = 0
    out.append(0)
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    out.append(0x00)
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i   = 0
    while i < len(data):
        code = data[i]
        if code == 0:
            return b''
        i += 1
        for _ in range(code - 1):
            if i >= len(data):
                return b''
            out.append(data[i])
            i += 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


FINDER_STATUS_FMT  = '<BBBHHBBB'
FINDER_STATUS_SIZE = struct.calcsize(FINDER_STATUS_FMT)  # 10

ERROR_LABELS = {0: 'OK', 1: 'WATCHDOG', 2: 'ENDSTOP', 3: 'BAD FRAME'}


# ── Motor controller ──────────────────────────────────────────────────────────

class MotorController:
    def __init__(self):
        self.serial_connection = None
        self.connected         = False

        self.focus_direction  = 0
        self.zoom_direction   = 0
        self.focus_speed      = 50
        self.zoom_speed       = 50
        self.pulse_mode       = False   # True = timed pulse, False = continuous
        self.pulse_duration_ms = 150    # ms per pulse in pulse mode

        self.status = {
            'flags': 0, 'endstops': 0, 'last_error': 0,
            'focus_active_ms': 0, 'zoom_active_ms': 0,
            'focus_pwm': 0, 'zoom_pwm': 0,
        }
        self._rx_buf = bytearray()

    def find_esp32_port(self):
        ports = serial.tools.list_ports.comports()
        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device} — {p.description}")
        for p in ports:
            if any(k in p.description.upper() for k in ("USB", "SERIAL", "CP210", "CH340")):
                print(f"Auto-selecting: {p.device}")
                return p.device
        if len(ports) == 1:
            return ports[0].device
        if not ports:
            print("No serial ports found!")
            return None
        try:
            choice = int(input("Enter port number: "))
            return ports[choice].device
        except (ValueError, IndexError):
            return None

    def connect(self, port=None):
        try:
            if port is None:
                port = self.find_esp32_port()
            if port is None:
                return False
            self.serial_connection = serial.Serial(port, BAUD_RATE, timeout=0)
            time.sleep(2)
            self.connected = True
            print(f"Connected to {port}")
            time.sleep(0.5)
            self.serial_connection.reset_input_buffer()
            return True
        except serial.SerialException as e:
            print(f"Connection error: {e}")
            self.connected = False
            return False

    def disconnect(self):
        if self.serial_connection and self.serial_connection.is_open:
            print("Stopping motors before disconnect...")
            self._send(0, 0)
            time.sleep(0.2)
            self.serial_connection.close()
            self.connected = False

    def _send(self, focus: int, zoom: int, duration_ms: int = 0):
        if not self.connected or not self.serial_connection:
            return
        focus       = max(-127, min(127, focus))
        zoom        = max(-127, min(127, zoom))
        duration_ms = max(0, min(65535, duration_ms))

        # Track pulse timing to avoid stomping it with keepalives
        if duration_ms > 0:
            self._pulse_end_time = time.time() + (duration_ms / 1000.0)
        else:
            self._pulse_end_time = 0

        fb, zb = focus & 0xFF, zoom & 0xFF
        data   = bytes([fb, zb]) + struct.pack('<H', duration_ms)
        payload = data + bytes([crc8(data)])
        try:
            self.serial_connection.write(cobs_encode(payload))
        except Exception as e:
            print(f"Send error: {e}")
            self.connected = False

    def update_motors(self):
        # If we have an active direction (e.g. from continuous mode or a running test),
        # we must send it.
        if self.focus_direction != 0 or self.zoom_direction != 0:
            self._send(
                self.focus_direction * self.focus_speed,
                self.zoom_direction  * self.zoom_speed,
            )
            return

        # If we are in pulse mode and idle, avoid stomping on a running pulse.
        if self.pulse_mode:
            now = time.time()
            # If a pulse is currently active on the ESP32, don't send anything.
            if now < getattr(self, '_pulse_end_time', 0):
                return

            # To avoid triggering the hardware watchdog while idle,
            # send a keepalive (0,0,0) every 250ms.
            if now < getattr(self, '_last_keepalive', 0) + 0.25:
                return
            self._last_keepalive = now

        # Continuous mode idle, or pulse mode idle (after pulse end/keepalive)
        self._send(0, 0)

    def read_serial(self):
        if not self.connected or not self.serial_connection:
            return
        try:
            waiting = self.serial_connection.in_waiting
            if waiting <= 0:
                return
            data = self.serial_connection.read(waiting)
        except Exception as e:
            print(f"Read error: {e}")
            self.connected = False
            return
        for b in data:
            if b == 0x00:
                if self._rx_buf:
                    self._parse_status(bytes(self._rx_buf))
                    self._rx_buf.clear()
            else:
                self._rx_buf.append(b)
                if len(self._rx_buf) > 32:
                    self._rx_buf.clear()

    def _parse_status(self, frame: bytes):
        decoded = cobs_decode(frame)
        if len(decoded) != FINDER_STATUS_SIZE:
            return
        fields = struct.unpack(FINDER_STATUS_FMT, decoded)
        flags, endstops, last_error, f_ms, z_ms, f_pwm, z_pwm, crc_recv = fields
        if crc_recv != crc8(decoded[:-1]):
            return
        self.status.update({
            'flags': flags, 'endstops': endstops, 'last_error': last_error,
            'focus_active_ms': f_ms, 'zoom_active_ms': z_ms,
            'focus_pwm': f_pwm, 'zoom_pwm': z_pwm,
        })

    @property
    def focus_active(self): return bool(self.status['flags'] & 0x01)
    @property
    def zoom_active(self):  return bool(self.status['flags'] & 0x02)
    @property
    def f_min(self): return bool(self.status['endstops'] & 0x01)
    @property
    def f_max(self): return bool(self.status['endstops'] & 0x02)
    @property
    def z_min(self): return bool(self.status['endstops'] & 0x04)
    @property
    def z_max(self): return bool(self.status['endstops'] & 0x08)


# ── Sweep test ────────────────────────────────────────────────────────────────

class SweepTest:
    """
    Endstop-to-endstop sweep test at three speeds, RETRIES runs each.
    Sequence per speed: (home → settle → sweep) × RETRIES, then average.
    Results: individual run times + average, saved to JSON on completion.
    """

    SPEEDS     = [('slow', 1), ('medium', 63), ('fast', 127)]
    RETRIES    = 3
    HOME_SPEED = 127
    SETTLE_MS  = 400
    TIMEOUT_MS = 25_000

    def __init__(self):
        self._reset()

    def _reset(self):
        self.running      = False
        self.done         = False
        self.motor        = None
        self.state        = 'idle'
        self.step_idx     = 0       # index into SPEEDS
        self.retry_idx    = 0       # retry counter for current speed
        self.current_runs = []      # elapsed_ms for retries of current speed
        self.state_start  = 0
        self.results      = []      # [(label, spd, [r1,r2,r3], avg_ms)]
        self.error        = None
        self.status_line  = ''

    def start(self, ctrl, motor: str):
        self._reset()
        self.motor       = motor
        self.running     = True
        self.state       = 'homing'
        self.state_start = pygame.time.get_ticks()
        self.status_line = 'Homing to MIN...'
        self._apply(ctrl)

    def cancel(self, ctrl):
        ctrl.focus_direction = 0
        ctrl.zoom_direction  = 0
        self._reset()

    # -- per-frame update -----------------------------------------------------

    def step(self, ctrl):
        if not self.running:
            return

        now     = pygame.time.get_ticks()
        elapsed = now - self.state_start
        at_min  = ctrl.f_min if self.motor == 'focus' else ctrl.z_min
        at_max  = ctrl.f_max if self.motor == 'focus' else ctrl.z_max

        n_speeds = len(self.SPEEDS)

        if self.state == 'homing':
            if at_min:
                self._go('settling', now, 'Settling...')
            elif elapsed > self.TIMEOUT_MS:
                self._fail(ctrl, 'Timeout: MIN endstop not reached')
                return

        elif self.state == 'settling':
            if elapsed >= self.SETTLE_MS:
                label, spd = self.SPEEDS[self.step_idx]
                run_num    = self.retry_idx + 1
                self._go('sweeping', now,
                    f'Speed {self.step_idx+1}/{n_speeds} {label}  run {run_num}/{self.RETRIES}')

        elif self.state == 'sweeping':
            if at_max:
                label, spd = self.SPEEDS[self.step_idx]
                self.current_runs.append(elapsed)
                self.retry_idx += 1

                if self.retry_idx < self.RETRIES:
                    # More retries for this speed — home again
                    self._go('homing', now, 'Homing to MIN...')
                else:
                    # All retries done — compute average, move to next speed
                    avg = int(sum(self.current_runs) / len(self.current_runs))
                    self.results.append((label, spd, list(self.current_runs), avg))
                    self.current_runs = []
                    self.retry_idx    = 0
                    self.step_idx    += 1

                    if self.step_idx >= n_speeds:
                        self.status_line = 'Complete!'
                        self._finish(ctrl)
                        return
                    else:
                        self._go('homing', now, 'Homing to MIN...')

            elif elapsed > self.TIMEOUT_MS:
                self._fail(ctrl, 'Timeout: MAX endstop not reached')
                return

        self._apply(ctrl)

    # -- helpers --------------------------------------------------------------

    def _go(self, state, now, text):
        self.state       = state
        self.state_start = now
        self.status_line = text

    def _apply(self, ctrl):
        is_focus = self.motor == 'focus'
        if self.state == 'homing':
            spd = self.HOME_SPEED
            if is_focus:
                ctrl.focus_direction, ctrl.focus_speed = -1, spd
                ctrl.zoom_direction = 0
            else:
                ctrl.zoom_direction, ctrl.zoom_speed = -1, spd
                ctrl.focus_direction = 0
        elif self.state == 'sweeping':
            _, spd = self.SPEEDS[self.step_idx]
            if is_focus:
                ctrl.focus_direction, ctrl.focus_speed = 1, spd
                ctrl.zoom_direction = 0
            else:
                ctrl.zoom_direction, ctrl.zoom_speed = 1, spd
                ctrl.focus_direction = 0
        else:  # settling / done
            ctrl.focus_direction = 0
            ctrl.zoom_direction  = 0

    def _save_results(self):
        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(os.path.dirname(__file__), f'sweep_{self.motor}_{ts}.json')
        avgs  = [r[3] for r in self.results if r[3] > 0]
        min_t = min(avgs) if avgs else 1
        data = {
            'timestamp': datetime.now().isoformat(),
            'motor':     self.motor,
            'retries':   self.RETRIES,
            'speeds': [
                {
                    'speed_label':  label,
                    'speed_val':    spd,
                    'pwm_est':      int(spd / 127 * (255 - 89) + 89) if spd > 0 else 0,
                    'runs_ms':      runs,
                    'avg_ms':       avg,
                    'relative_pct': int(min_t / avg * 100) if avg > 0 else 0,
                }
                for label, spd, runs, avg in self.results
            ],
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Results saved to {filename}")
        self.saved_path = filename

    def _finish(self, ctrl):
        self.running         = False
        self.done            = True
        ctrl.focus_direction = 0
        ctrl.zoom_direction  = 0
        if self.results and not self.error:
            self._save_results()

    def _fail(self, ctrl, msg):
        self.error       = msg
        self.status_line = f'ERROR: {msg}'
        self._finish(ctrl)


# ── UI helpers ────────────────────────────────────────────────────────────────

def draw_endstop(surface, font, x, y, label, triggered):
    pygame.draw.circle(surface, RED if triggered else GRAY, (x, y + 8), 8)
    surface.blit(font.render(label, True, WHITE), (x + 14, y))


def draw_arrow_h(surface, color, cx, cy, direction):
    d   = direction
    pts = [(cx + d*14, cy), (cx - d*6, cy - 10), (cx - d*6, cy + 10)]
    pygame.draw.polygon(surface, color, pts)


def draw_arrow_v(surface, color, cx, cy, direction):
    d   = direction
    pts = [(cx, cy - d*14), (cx - 10, cy + d*6), (cx + 10, cy + d*6)]
    pygame.draw.polygon(surface, color, pts)


def draw_test_panel(surface, fonts, test: SweepTest, W):
    """Draw the sweep test results panel."""
    font_md, font_sm = fonts

    RETRIES = SweepTest.RETRIES
    PX, PY, PW, PH = 30, 160, W - 60, 330
    panel = pygame.Surface((PW, PH), pygame.SRCALPHA)
    panel.fill((18, 18, 30, 230))
    surface.blit(panel, (PX, PY))
    pygame.draw.rect(surface, BLUE, (PX, PY, PW, PH), 1)

    motor_label = test.motor.upper() if test.motor else ''
    title = font_md.render(f"{motor_label} SWEEP TEST  ({RETRIES} runs/speed)", True, YELLOW)
    surface.blit(title, (PX + PW // 2 - title.get_width() // 2, PY + 8))

    # Status line
    status_col = RED if test.error else (GREEN if test.done else WHITE)
    surface.blit(font_sm.render(test.status_line, True, status_col), (PX + 14, PY + 36))

    # Progress: speed dots with retry sub-dots
    for i, (lbl, _) in enumerate(SweepTest.SPEEDS):
        done_spd = i < len(test.results)
        curr_spd = (i == len(test.results)) and test.running
        dot_col  = GREEN if done_spd else (YELLOW if curr_spd else GRAY)
        cx = PX + 14 + i * (RETRIES * 14 + 20)
        pygame.draw.circle(surface, dot_col, (cx, PY + 56), 6)
        surface.blit(font_sm.render(lbl[0].upper(), True, dot_col), (cx + 9, PY + 50))
        # retry sub-dots
        for r in range(RETRIES):
            done_r = done_spd or (curr_spd and r < test.retry_idx)
            rcol   = GREEN if done_r else (YELLOW if (curr_spd and r == test.retry_idx) else GRAY)
            pygame.draw.circle(surface, rcol, (cx + 22 + r * 12, PY + 56), 4)

    # Column headers:  Speed | Val | R1 | R2 | R3 | Avg | Rel.
    run_labels = [f'R{i+1}' for i in range(RETRIES)]
    headers    = ['Speed', 'Val'] + run_labels + ['Avg ms', 'Rel.']
    # x positions — distribute across PW
    hx = [PX + 14, PX + 68] + [PX + 118 + i * 62 for i in range(RETRIES)] + \
         [PX + 118 + RETRIES * 62, PX + 118 + RETRIES * 62 + 74]
    hy = PY + 72
    for txt, x in zip(headers, hx):
        surface.blit(font_sm.render(txt, True, GRAY), (x, hy))
    pygame.draw.line(surface, GRAY, (PX + 10, hy + 18), (PX + PW - 10, hy + 18), 1)

    # Completed result rows
    avgs  = [r[3] for r in test.results if r[3] > 0]
    min_t = min(avgs) if avgs else 1

    for row, (label, spd, runs, avg) in enumerate(test.results):
        ry      = hy + 24 + row * 30
        rel     = int(min_t / avg * 100) if avg > 0 else 0
        bar_w   = int(rel / 100 * 60)
        vals    = [label, str(spd)] + [str(r) for r in runs] + [str(avg), '']
        for val, x in zip(vals, hx):
            surface.blit(font_sm.render(val, True, WHITE), (x, ry))
        pygame.draw.rect(surface, DARK_GRAY, (hx[-1], ry + 2, 60, 14))
        pygame.draw.rect(surface, GREEN,     (hx[-1], ry + 2, bar_w, 14))
        surface.blit(font_sm.render(f'{rel}%', True, WHITE), (hx[-1] + 66, ry))

    # In-progress row (current speed, partial retries)
    curr_row = len(test.results)
    if test.running and curr_row < len(SweepTest.SPEEDS):
        ry    = hy + 24 + curr_row * 30
        label, spd = SweepTest.SPEEDS[curr_row]
        partial    = test.current_runs + ['…'] * (RETRIES - len(test.current_runs))
        vals       = [label, str(spd)] + [str(v) for v in partial] + ['…', '']
        for val, x in zip(vals, hx):
            col = YELLOW if val == '…' else WHITE
            surface.blit(font_sm.render(val, True, col), (x, ry))

    # Pending rows
    for row in range(curr_row + (1 if test.running else 0), len(SweepTest.SPEEDS)):
        ry    = hy + 24 + row * 30
        label = SweepTest.SPEEDS[row][0]
        surface.blit(font_sm.render(label, True, GRAY), (hx[0], ry))
        for x in hx[2:]:
            surface.blit(font_sm.render('—', True, GRAY), (x, ry))

    # Saved path
    if test.done and not test.error and hasattr(test, 'saved_path'):
        surface.blit(
            font_sm.render(f"Saved: {os.path.basename(test.saved_path)}", True, GRAY),
            (PX + 14, PY + PH - 38))

    # Dismiss hint
    hint_col = WHITE if test.done or test.error else GRAY
    surface.blit(
        font_sm.render(
            'ESC to cancel' if test.running else 'T/Y to rerun  ESC to dismiss',
            True, hint_col),
        (PX + PW // 2 - font_sm.size('T/Y to rerun  ESC to dismiss')[0] // 2, PY + PH - 20))


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    pygame.init()
    W, H   = 640, 520
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Motorized Finder Controller")

    font_lg = pygame.font.Font(None, 42)
    font_md = pygame.font.Font(None, 30)
    font_sm = pygame.font.Font(None, 22)

    ctrl = MotorController()
    test = SweepTest()

    print("\n" + "=" * 50)
    print("Motorized Finder Controller")
    print("=" * 50)

    if not ctrl.connect():
        print("Failed to connect. Exiting.")
        pygame.quit()
        sys.exit(1)

    keys_held = {
        pygame.K_RIGHT: False,
        pygame.K_LEFT:  False,
        pygame.K_UP:    False,
        pygame.K_DOWN:  False,
    }

    clock   = pygame.time.Clock()
    running = True

    print("\nControls:")
    print("  ←/→  Focus    ↑/↓  Zoom    [/]  focus speed    ;/'  zoom speed")
    print("  T  Focus sweep test    Y  Zoom sweep test    ESC  Exit")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    if test.running:
                        test.cancel(ctrl)
                    elif test.done or test.error:
                        test._reset()
                    else:
                        running = False

                # Sweep tests
                elif event.key == pygame.K_t:
                    test.start(ctrl, 'focus')

                elif event.key == pygame.K_y:
                    test.start(ctrl, 'zoom')

                # Movement keys — blocked while test running
                elif not test.running:
                    if event.key == pygame.K_p:
                        ctrl.pulse_mode = not ctrl.pulse_mode
                        # Clear directions when switching to pulse mode to avoid carry-over
                        if ctrl.pulse_mode:
                            ctrl.focus_direction = 0
                            ctrl.zoom_direction = 0
                            for k in keys_held: keys_held[k] = False
                        mode = f"PULSE ({ctrl.pulse_duration_ms}ms)" if ctrl.pulse_mode else "CONTINUOUS"
                        print(f"Motor mode → {mode}")
                    elif event.key == pygame.K_MINUS:
                        ctrl.pulse_duration_ms = max(10, ctrl.pulse_duration_ms - 10)
                        print(f"Pulse duration → {ctrl.pulse_duration_ms}ms")
                    elif event.key == pygame.K_EQUALS:
                        ctrl.pulse_duration_ms = min(2000, ctrl.pulse_duration_ms + 10)
                        print(f"Pulse duration → {ctrl.pulse_duration_ms}ms")
                    elif ctrl.pulse_mode:
                        # Pulse mode: one timed command per keydown, no hold repeat needed
                        if event.key == pygame.K_RIGHT:
                            ctrl._send(ctrl.focus_speed, 0, ctrl.pulse_duration_ms)
                        elif event.key == pygame.K_LEFT:
                            ctrl._send(-ctrl.focus_speed, 0, ctrl.pulse_duration_ms)
                        elif event.key == pygame.K_UP:
                            ctrl._send(0, ctrl.zoom_speed, ctrl.pulse_duration_ms)
                        elif event.key == pygame.K_DOWN:
                            ctrl._send(0, -ctrl.zoom_speed, ctrl.pulse_duration_ms)
                        elif event.key == pygame.K_LEFTBRACKET:
                            ctrl.focus_speed = max(1, ctrl.focus_speed - 10)
                        elif event.key == pygame.K_RIGHTBRACKET:
                            ctrl.focus_speed = min(127, ctrl.focus_speed + 10)
                        elif event.key == pygame.K_SEMICOLON:
                            ctrl.zoom_speed = max(1, ctrl.zoom_speed - 10)
                        elif event.key == pygame.K_QUOTE:
                            ctrl.zoom_speed = min(127, ctrl.zoom_speed + 10)
                    else:
                        # Continuous mode: hold keys for sustained movement
                        if event.key == pygame.K_RIGHT and not keys_held[pygame.K_RIGHT]:
                            keys_held[pygame.K_RIGHT] = True
                            ctrl.focus_direction = 1
                            ctrl.update_motors()
                        elif event.key == pygame.K_LEFT and not keys_held[pygame.K_LEFT]:
                            keys_held[pygame.K_LEFT] = True
                            ctrl.focus_direction = -1
                            ctrl.update_motors()
                        elif event.key == pygame.K_UP and not keys_held[pygame.K_UP]:
                            keys_held[pygame.K_UP] = True
                            ctrl.zoom_direction = 1
                            ctrl.update_motors()
                        elif event.key == pygame.K_DOWN and not keys_held[pygame.K_DOWN]:
                            keys_held[pygame.K_DOWN] = True
                            ctrl.zoom_direction = -1
                            ctrl.update_motors()
                        elif event.key == pygame.K_LEFTBRACKET:
                            ctrl.focus_speed = max(1, ctrl.focus_speed - 10)
                            if ctrl.focus_direction != 0:
                                ctrl.update_motors()
                        elif event.key == pygame.K_RIGHTBRACKET:
                            ctrl.focus_speed = min(127, ctrl.focus_speed + 10)
                            if ctrl.focus_direction != 0:
                                ctrl.update_motors()
                        elif event.key == pygame.K_SEMICOLON:
                            ctrl.zoom_speed = max(1, ctrl.zoom_speed - 10)
                            if ctrl.zoom_direction != 0:
                                ctrl.update_motors()
                        elif event.key == pygame.K_QUOTE:
                            ctrl.zoom_speed = min(127, ctrl.zoom_speed + 10)
                            if ctrl.zoom_direction != 0:
                                ctrl.update_motors()

            elif event.type == pygame.KEYUP and not test.running and not ctrl.pulse_mode:
                if event.key == pygame.K_RIGHT and keys_held[pygame.K_RIGHT]:
                    keys_held[pygame.K_RIGHT] = False
                    ctrl.focus_direction = 0
                    ctrl.update_motors()
                elif event.key == pygame.K_LEFT and keys_held[pygame.K_LEFT]:
                    keys_held[pygame.K_LEFT] = False
                    ctrl.focus_direction = 0
                    ctrl.update_motors()
                elif event.key == pygame.K_UP and keys_held[pygame.K_UP]:
                    keys_held[pygame.K_UP] = False
                    ctrl.zoom_direction = 0
                    ctrl.update_motors()
                elif event.key == pygame.K_DOWN and keys_held[pygame.K_DOWN]:
                    keys_held[pygame.K_DOWN] = False
                    ctrl.zoom_direction = 0
                    ctrl.update_motors()

        # Advance test state machine
        test.step(ctrl)

        # Continuous motor command (watchdog keepalive + sustained movement)
        ctrl.update_motors()

        # Read incoming FinderStatus frames
        ctrl.read_serial()

        # ── Draw ─────────────────────────────────────────────────────────────
        screen.fill(BLACK)

        # Title
        t = font_lg.render("Finder Controller", True, WHITE)
        screen.blit(t, (W // 2 - t.get_width() // 2, 12))

        # Connection + error
        conn_txt = font_sm.render(
            'Connected' if ctrl.connected else 'Disconnected',
            True, GREEN if ctrl.connected else RED)
        screen.blit(conn_txt, (20, 58))

        err_code  = ctrl.status['last_error']
        err_label = ERROR_LABELS.get(err_code, f'ERR {err_code}')
        err_color = GREEN if err_code == 0 else (YELLOW if err_code == 2 else RED)
        err_txt   = font_sm.render(f"Status: {err_label}", True, err_color)
        screen.blit(err_txt, (W - err_txt.get_width() - 20, 58))

        # ── FOCUS section ─────────────────────────────────────────────────────
        pygame.draw.line(screen, DARK_GRAY, (20, 82), (W - 20, 82), 1)

        screen.blit(font_md.render("FOCUS", True, WHITE), (20, 90))
        fstate_txt = font_md.render(
            "ACTIVE" if ctrl.focus_active else "STOPPED",
            True, GREEN if ctrl.focus_active else GRAY)
        screen.blit(fstate_txt, (110, 90))
        screen.blit(font_sm.render(
            f"speed {ctrl.focus_speed}/127  ([/] to adjust)", True, BLUE), (270, 94))

        left_col  = BLUE if keys_held[pygame.K_LEFT]  else GRAY
        right_col = BLUE if keys_held[pygame.K_RIGHT] else GRAY
        draw_arrow_h(screen, left_col,  80,  145, -1)
        draw_arrow_h(screen, right_col, 560, 145,  1)

        dir_lbl = {-1: "← BACK", 0: "STOPPED", 1: "FWD →"}.get(ctrl.focus_direction, "")
        dir_txt = font_md.render(dir_lbl, True, WHITE if ctrl.focus_direction else GRAY)
        screen.blit(dir_txt, (W // 2 - dir_txt.get_width() // 2, 136))

        pwm_w = int(ctrl.status['focus_pwm'] / 255 * 200)
        pygame.draw.rect(screen, DARK_GRAY, (210, 160, 200, 10))
        pygame.draw.rect(screen, GREEN,     (210, 160, pwm_w, 10))
        screen.blit(font_sm.render(f"PWM {ctrl.status['focus_pwm']}", True, GRAY), (418, 157))

        draw_endstop(screen, font_sm, 120, 178, "F-MIN", ctrl.f_min)
        draw_endstop(screen, font_sm, 250, 178, "F-MAX", ctrl.f_max)
        screen.blit(font_sm.render(
            f"active {ctrl.status['focus_active_ms']} ms", True, GRAY), (420, 178))

        # ── ZOOM section ──────────────────────────────────────────────────────
        pygame.draw.line(screen, DARK_GRAY, (20, 210), (W - 20, 210), 1)

        screen.blit(font_md.render("ZOOM", True, WHITE), (20, 218))
        zstate_txt = font_md.render(
            "ACTIVE" if ctrl.zoom_active else "STOPPED",
            True, GREEN if ctrl.zoom_active else GRAY)
        screen.blit(zstate_txt, (110, 218))
        screen.blit(font_sm.render(
            f"speed {ctrl.zoom_speed}/127  (;/' to adjust)", True, BLUE), (270, 222))

        up_col   = BLUE if keys_held[pygame.K_UP]   else GRAY
        down_col = BLUE if keys_held[pygame.K_DOWN] else GRAY
        draw_arrow_v(screen, up_col,   80, 268, +1)
        draw_arrow_v(screen, down_col, 80, 310, -1)

        zdir_lbl = {-1: "↓ BACK", 0: "STOPPED", 1: "FWD ↑"}.get(ctrl.zoom_direction, "")
        zdir_txt = font_md.render(zdir_lbl, True, WHITE if ctrl.zoom_direction else GRAY)
        screen.blit(zdir_txt, (W // 2 - zdir_txt.get_width() // 2, 280))

        zpwm_w = int(ctrl.status['zoom_pwm'] / 255 * 200)
        pygame.draw.rect(screen, DARK_GRAY, (210, 300, 200, 10))
        pygame.draw.rect(screen, GREEN,     (210, 300, zpwm_w, 10))
        screen.blit(font_sm.render(f"PWM {ctrl.status['zoom_pwm']}", True, GRAY), (418, 297))

        draw_endstop(screen, font_sm, 120, 318, "Z-MIN", ctrl.z_min)
        draw_endstop(screen, font_sm, 250, 318, "Z-MAX", ctrl.z_max)
        screen.blit(font_sm.render(
            f"active {ctrl.status['zoom_active_ms']} ms", True, GRAY), (420, 318))

        # ── Endstop summary ───────────────────────────────────────────────────
        pygame.draw.line(screen, DARK_GRAY, (20, 350), (W - 20, 350), 1)
        screen.blit(font_sm.render("Endstops:", True, GRAY), (20, 358))
        draw_endstop(screen, font_sm, 110, 355, "F-MIN", ctrl.f_min)
        draw_endstop(screen, font_sm, 210, 355, "F-MAX", ctrl.f_max)
        draw_endstop(screen, font_sm, 310, 355, "Z-MIN", ctrl.z_min)
        draw_endstop(screen, font_sm, 410, 355, "Z-MAX", ctrl.z_max)

        # ── Pulse mode indicator ──────────────────────────────────────────────
        pygame.draw.line(screen, DARK_GRAY, (20, 378), (W - 20, 378), 1)
        if ctrl.pulse_mode:
            mode_str = f"PULSE MODE  {ctrl.pulse_duration_ms}ms  (-/= adjust)"
            mode_col = YELLOW
        else:
            mode_str = "CONTINUOUS MODE"
            mode_col = BLUE
        screen.blit(font_sm.render(mode_str, True, mode_col), (20, 386))
        screen.blit(font_sm.render("P = toggle mode", True, GRAY), (W - 160, 386))

        # ── Key hints ─────────────────────────────────────────────────────────
        pygame.draw.line(screen, DARK_GRAY, (20, 406), (W - 20, 406), 1)
        hints = [
            ("←/→", "focus"), ("↑/↓", "zoom"),
            ("[/]", "focus spd"), (";/'", "zoom spd"),
            ("T", "focus test"), ("Y", "zoom test"), ("ESC", "exit"),
        ]
        hx = 20
        for key, desc in hints:
            kt = font_sm.render(key, True, YELLOW)
            dt = font_sm.render(f" {desc}   ", True, GRAY)
            screen.blit(kt, (hx, 414))
            screen.blit(dt, (hx + kt.get_width(), 414))
            hx += kt.get_width() + dt.get_width()

        # ── Test panel overlay ────────────────────────────────────────────────
        if test.running or test.done or test.error:
            draw_test_panel(screen, (font_md, font_sm), test, W)

        pygame.display.flip()
        clock.tick(60)

    ctrl.disconnect()
    pygame.quit()
    print("Exited.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pygame.quit()
        sys.exit(0)
