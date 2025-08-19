#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Layer 5 — Decision Maker for Human-Follow (Ackermann crawler) with directional loss recovery.

WHAT THIS DOES
- Runs a state machine (INIT → SEARCH → ACQUIRE → FOLLOW → FAULT).
- Reads latest vision (offset_x, area, quality, stamp_ms) from VisionBus.
- Issues short ARC/DRIVE impulses via Layer 4.
- NEW: When the target is lost, briefly "peek" toward the side the human vanished on,
       then stop/idle if not reacquired in time.
"""

from __future__ import annotations
import time
import threading
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Dict, Any

# ---- import your Layer 4 (must be in same folder) ----
from rc_layer4 import RcLayer4, L4Error


# ---------------------------- Parameters ----------------------------

@dataclass
class HFParams:
    # Steering (ARC)
    deadband_x: float = 0.08          # |offset_x| below this => no steering action
    k_arc_pwm:   int  = 900           # scale factor from |x| to PWM
    min_arc_pwm: int  = 120
    max_arc_pwm: int  = 220
    arc_impulse_ms: int = 220

    # Distance (DRIVE)
    target_area: float = 0.10         # desired bbox area fraction
    area_tol:    float = 0.03         # tolerance band around target_area
    k_drive_pwm: int   = 2400
    min_drive_pwm: int = 120
    max_drive_pwm: int = 210
    drive_impulse_ms: int = 260

    # Back-off when too close
    close_back_pwm: int = 140
    close_back_ms:  int = 150

    # Freshness & loop
    stale_ms:   int   = 300           # if measurement older than this => treat as stale
    lost_ms:    int   = 700           # time without fresh detection to declare LOST/SEARCH
    loop_hz:    float = 15.0          # control loop rate (Hz)
    cooldown_ms:int   = 60            # cooldown after issuing a command

    # Filters (1-pole IIR)
    tau_x_ms:   int = 180
    tau_area_ms:int = 220

    # Logging
    verbose: bool = True

    # ---------- NEW: directional recovery on loss ----------
    recover_enable:      bool = True   # turn-on directional recovery
    recover_max_ms:      int  = 3000   # total budget to keep peeking (ms)
    recover_arc_pwm:     int  = 250    # strength of each peek arc
    recover_impulse_ms:  int  = 180    # duration of each arc impulse
    recover_cooldown_ms: int  = 120    # gap between peeks
    recover_min_abs_x:   float = 0.10  # only recover if last |x| exceeded this
    quality_gate:        float = 0.60  # minimum confidence to consider “good”


# ---------------------------- Vision plumbing ----------------------------

@dataclass
class VisionSample:
    offset_x: float     # [-1..+1], negative left, positive right
    area:    float      # [0..1] bbox_area / frame_area
    quality: float      # [0..1]
    stamp_ms:int        # monotonic milliseconds


class VisionBus:
    """
    Minimal thread-safe mailbox for the latest vision measurement.
    Camera adapter calls publish(); Layer 5 polls latest().
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._latest: Optional[VisionSample] = None

    def publish(self, offset_x: float, area: float, quality: float):
        now_ms = int(time.monotonic() * 1000)
        sample = VisionSample(offset_x, area, quality, now_ms)
        with self._lock:
            self._latest = sample

    def latest(self) -> Optional[VisionSample]:
        with self._lock:
            return self._latest


# ---------------------------- Synthetic Vision (unchanged) ----------------------------

class SyntheticVision(threading.Thread):
    def __init__(self, bus: VisionBus, period_s: float = 0.05):
        super().__init__(daemon=True)
        self.bus = bus
        self.period_s = period_s
        self._stop_evt = threading.Event()

    def stop(self): self._stop_evt.set()

    def run(self):
        t0 = time.monotonic()
        while not self._stop_evt.is_set():
            t = time.monotonic() - t0
            offset_x = 0.25 * (1 if int(t) % 8 < 4 else -1) + 0.12 * (((t * 0.8) % 1.0) - 0.5) * 2
            area     = 0.06 + 0.02 * ((t * 0.3) % 1.0)
            quality  = 0.9
            self.bus.publish(offset_x, area, quality)
            time.sleep(self.period_s)


# ---------------------------- Layer 5 Controller ----------------------------

class Mode(Enum):
    INIT = auto()
    SEARCH = auto()
    ACQUIRE = auto()
    FOLLOW = auto()
    FAULT = auto()


def _clamp(v, lo, hi): return hi if v > hi else lo if v < lo else v


class Layer5(threading.Thread):
    """
    Decision layer. One command at a time via L4. Run as its own thread.
    """
    def __init__(self, l4: RcLayer4, vision: VisionBus, params: Optional[HFParams] = None):
        super().__init__(daemon=True)
        self.l4 = l4
        self.vision = vision
        self.p = params or HFParams()

        # internal state
        self.mode = Mode.INIT
        self._stop_evt = threading.Event()
        self._cooldown_until_ms = 0
        self._last_seen_ms = 0
        self._centered_once = False
        self._search_stopped = False

        # filters
        self._x_f = 0.0
        self._area_f = 0.0
        self._have_filter_seed = False

        # -------- NEW: directional recovery memory --------
        self._last_good_x: float = 0.0         # sign tells last-seen side
        self._recover_active: bool = False
        self._recover_until_ms: int = 0        # wall-clock deadline for recovery peeks

    def stop(self): self._stop_evt.set()

    # --------- helpers ---------

    def _now_ms(self) -> int:
        return int(time.monotonic() * 1000)

    def _iir(self, prev: float, new: float, tau_ms: int, dt_ms: int) -> float:
        if tau_ms <= 0 or dt_ms <= 0: return new
        a = dt_ms / (tau_ms + dt_ms)
        return (1 - a) * prev + a * new

    def _arc_pwm(self, abs_x: float) -> int:
        return int(_clamp(abs_x * self.p.k_arc_pwm, self.p.min_arc_pwm, self.p.max_arc_pwm))

    def _drive_pwm(self, err_area: float) -> int:
        return int(_clamp(err_area * self.p.k_drive_pwm, self.p.min_drive_pwm, self.p.max_drive_pwm))

    def _log(self, *a):
        if self.p.verbose: print(*a)

    # --- fixed: local cooldown override, never forwarded into L4 ---
    def _issue(self, cmd: str, *args, cooldown_override_ms: Optional[int] = None, **kw):
        """Single actuation to L4 with OK/DONE wait + cooldown (optionally overridden)."""
        try:
            res = self.l4.send(cmd, *args, **kw)     # do NOT pass the override into L4
            cool = self.p.cooldown_ms if (cooldown_override_ms is None) else int(cooldown_override_ms)
            self._cooldown_until_ms = self._now_ms() + max(0, cool)
            return res
        except L4Error as e:
            self._log(f"[FAULT] L4 error on {cmd}: {e}")
            self.mode = Mode.FAULT
            try:
                self.l4.send("stop")
            except Exception:
                pass
            return None


    # --------- main loop ---------

    def run(self):
        period_s = 1.0 / max(self.p.loop_hz, 1.0)
        last_loop_ms = self._now_ms()

        while not self._stop_evt.is_set():
            t_ms = self._now_ms()

            # Fetch latest vision
            m = self.vision.latest()
            fresh = False
            if m:
                age = t_ms - m.stamp_ms
                fresh = (age <= self.p.stale_ms)
                if fresh:
                    self._last_seen_ms = t_ms

                    # seed filters on first sample
                    if not self._have_filter_seed:
                        self._x_f, self._area_f = m.offset_x, m.area
                        self._have_filter_seed = True

                    # update filters
                    dt_ms = max(t_ms - last_loop_ms, 1)
                    self._x_f    = self._iir(self._x_f,    m.offset_x, self.p.tau_x_ms,    dt_ms)
                    self._area_f = self._iir(self._area_f, m.area,     self.p.tau_area_ms, dt_ms)

                    # remember last-good side (only if confident)
                    if m.quality >= self.p.quality_gate:
                        self._last_good_x = self._x_f

            # Loss detection (FOLLOW/ACQUIRE → SEARCH + center + arm recovery)
            if self.mode in (Mode.FOLLOW, Mode.ACQUIRE):
                if t_ms - self._last_seen_ms > self.p.lost_ms:
                    self._log("[SEARCH] Lost target → stop + center, arm directional recovery")
                    self._issue("stop")
                    self._issue("center")
                    self._centered_once = True
                    self._search_stopped = True
                    self.mode = Mode.SEARCH

                    # Arm recovery toward last seen side, if meaningful
                    self._recover_active = False
                    if self.p.recover_enable and abs(self._last_good_x) >= self.p.recover_min_abs_x:
                        self._recover_active = True
                        self._recover_until_ms = t_ms + max(0, self.p.recover_max_ms)

            # Cooldown gate
            if t_ms < self._cooldown_until_ms:
                time.sleep(0.002)
                last_loop_ms = t_ms
                continue

            # ---------------- State machine ----------------
            if self.mode == Mode.INIT:
                self._log("[INIT] Centering, then SEARCH")
                self._issue("stop")
                self._issue("center")
                self.mode = Mode.SEARCH
                self._search_stopped = True
                self._recover_active = False

            elif self.mode == Mode.SEARCH:
                # If we see a fresh confident target, acquire immediately.
                if fresh and m and m.quality >= self.p.quality_gate:
                    self._log("[ACQUIRE] Fresh target seen")
                    self._search_stopped = False
                    self._recover_active = False
                    self.mode = Mode.ACQUIRE
                else:
                    # Directional recovery peeks (ONE-SIDED) while budget remains
                    if self._recover_active and t_ms < self._recover_until_ms:
                        # ensure stop-once
                        if not self._search_stopped:
                            self._issue("stop")
                            self._search_stopped = True

                        # Decide peek direction from last_good_x
                        dir_cmd = "arcleft" if self._last_good_x < 0 else "arcright"
                        self._log(f"[SEARCH/RECOVER] peek {dir_cmd} pwm={self.p.recover_arc_pwm}")
                        # Arc pulse with its own cooldown so we peek rhythmically
                        self._issue(
                            dir_cmd,
                            int(_clamp(self.p.recover_arc_pwm, self.p.min_arc_pwm, self.p.max_arc_pwm)),
                            int(self.p.recover_impulse_ms),
                            end="U",
                            cooldown_override_ms=int(self.p.recover_cooldown_ms),
                        )
                    else:
                        # Exhausted recovery → idle stop (no sweeping)
                        if not self._search_stopped:
                            self._log("[SEARCH] no target → STOP (idle)")
                            self._issue("stop")
                            self._search_stopped = True
                        # remain idle

            elif self.mode == Mode.ACQUIRE:
                if not fresh or (m and m.quality < self.p.quality_gate):
                    self._log("[SEARCH] Lost during acquire")
                    self._issue("stop")
                    self._search_stopped = True
                    self.mode = Mode.SEARCH
                    # Arm recovery too, if we had a meaningful last_good_x
                    if self.p.recover_enable and abs(self._last_good_x) >= self.p.recover_min_abs_x:
                        self._recover_active = True
                        self._recover_until_ms = t_ms + max(0, self.p.recover_max_ms)
                else:
                    # Nudge heading toward target
                    pwm = self._arc_pwm(abs(self._x_f))
                    dir_cmd = "arcleft" if self._x_f < 0 else "arcright"
                    self._log(f"[ACQUIRE] {dir_cmd} pwm={pwm}")
                    self._issue(dir_cmd, pwm, self.p.arc_impulse_ms, end="U")

                    # If clearly far, add a short forward
                    err = self.p.target_area - self._area_f
                    if err > self.p.area_tol:
                        fwd = self._drive_pwm(err)
                        self._log(f"[ACQUIRE] forward pwm={fwd}")
                        self._issue("driveforward", fwd, self.p.drive_impulse_ms)

                    self.mode = Mode.FOLLOW

            elif self.mode == Mode.FOLLOW:
                if not fresh or (m and m.quality < self.p.quality_gate):
                    # stale → stop once, maybe center once
                    if not self._search_stopped:
                        self._issue("stop")
                        self._search_stopped = True
                    if not self._centered_once:
                        self._log("[FOLLOW] stale → center once")
                        self._issue("center")
                        self._centered_once = True
                    # Note: recovery arming happens in the loss block above
                else:
                    self._centered_once = False
                    self._search_stopped = False

                    # Priority 1: steering
                    if abs(self._x_f) > self.p.deadband_x:
                        pwm = self._arc_pwm(abs(self._x_f))
                        dir_cmd = "arcleft" if self._x_f < 0 else "arcright"
                        self._log(f"[FOLLOW] steer {dir_cmd} pwm={pwm}")
                        self._issue(dir_cmd, pwm, self.p.arc_impulse_ms, end="U")
                    else:
                        # Priority 2: distance
                        err = self.p.target_area - self._area_f
                        if err > self.p.area_tol:
                            pwm = self._drive_pwm(err)
                            self._log(f"[FOLLOW] forward pwm={pwm}")
                            self._issue("driveforward", pwm, self.p.drive_impulse_ms)
                        elif err < -self.p.area_tol:
                            self._log(f"[FOLLOW] back pwm={self.p.close_back_pwm}")
                            self._issue("driveback", self.p.close_back_pwm, self.p.close_back_ms)
                        else:
                            if not self._centered_once:
                                self._log("[FOLLOW] in band → center")
                                self._issue("center")
                                self._centered_once = True

            elif self.mode == Mode.FAULT:
                self._log("[FAULT] stopping and recovering to SEARCH")
                try: self.l4.send("stop")
                except Exception: pass
                time.sleep(0.1)
                self.mode = Mode.SEARCH
                self._search_stopped = True
                self._recover_active = False

            last_loop_ms = t_ms
            time.sleep(period_s)


# ---------------------------- Inline demo (unchanged) ----------------------------

if __name__ == "__main__":
    l4 = RcLayer4()
    l4.open()

    bus = VisionBus()

    # Prefer PiNSIGHT if available; otherwise SyntheticVision. (Adapter publishes to VisionBus.)
    try:
        from vision_pinsight import PiNSightDepthAI
        cam = PiNSightDepthAI(bus, blob_path=None, conf_threshold=0.5, fps=30)
        cam.start()
        camera_thread = cam
    except Exception as e:
        print(f"[WARN] PiNSIGHT adapter not available ({e}). Falling back to SyntheticVision.")
        synth = SyntheticVision(bus, period_s=0.05)
        synth.start()
        camera_thread = synth

    params = HFParams(verbose=True)
    l5 = Layer5(l4, bus, params)
    l5.start()

    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        l5.stop()
        camera_thread.stop()
        l5.join(timeout=1.0)
        camera_thread.join(timeout=1.0)
        l4.close()
