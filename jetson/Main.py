#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
l5_check.py — Validate Layer 5 decision logic with real PiNSIGHT camera input.

WHAT THIS DOES
- Starts Layer 5 (from rc_layer5.py) and feeds it vision.
- Vision priority:
    1) PiNSIGHT camera via DepthAI (on-device MobileNet-SSD)
    2) Fallback to SyntheticVision if camera/model not available
- In STUB mode: FakeRcLayer4 (no hardware) + real camera input → safe, no motion.
- In LIVE mode: RcLayer4 talks to Teensy + real camera input → wheels up on a stand!

REQUIREMENTS
- rc_layer4.py, rc_layer5.py in the same folder.
- vision_pinsight.py (the adapter) in the same folder.
- pip install depthai (for the camera), and a MobileNet-SSD .blob file.

HOW TO RUN
- Edit CONFIG below (MODE/VISION/BLOB_PATH/PORT) and run:  python l5_check.py
"""

import time
import threading
from typing import Optional, Dict, Any

# ----------------------- CONFIG -----------------------
MODE = "LIVE"            # "STUB" (no robot motion) or "LIVE" (real Teensy)
VISION = "CAM"           # "CAM" uses PiNSIGHT/DepthAI, "FAKE" uses SyntheticVision
PORT: Optional[str] = None   # LIVE only: None = auto-detect Teensy, or set "COM7" / "/dev/ttyACM0"
RUN_SECONDS = 0          # 0 or <=0 = run until Ctrl+C; otherwise stop after N seconds
PRINT_TELEM = True       # LIVE: print TELEM (throttled). STUB: ignored.

# DepthAI MobileNet-SSD blob path (None = let adapter auto-find common locations)
BLOB_PATH: Optional[str] = None
CONF_THRESHOLD = 0.5     # detector confidence
FPS = 30                 # camera NN fps
SHOW_PREVIEW = True      # show live camera window with overlays (if display & cv2 available)

# --- follow-distance tuning (Layer-5) ---
TARGET_AREA     = 0.18   # bigger = follows closer, smaller = farther
AREA_TOL        = 0.040   # widen to reduce yo-yo, tighten to be snappier
DRIVE_K         = 2200    # drive aggression toward the set-point
MAX_DRIVE_PWM   = 190     # cap forward speed (safety)
DRIVE_MS        = 240     # duration of each forward impulse (ms)


# ------------------------------------------------------

# Import Layer 5 bits
from rc_layer5 import Layer5, VisionBus, SyntheticVision, HFParams

# In LIVE we import your real Layer 4. In STUB we provide a fake with same API.
if MODE.upper() == "LIVE":
    from rc_layer4 import RcLayer4, L4Error  # real serial client
else:
    # Minimal stand-in error type to satisfy Layer5's try/except (not actually used in STUB)
    class L4Error(RuntimeError): ...
    class RcLayer4:
        """NOT USED in STUB. Here only so imports won't fail if someone switches MODE dynamically."""
        pass


# ----------------------- STUB L4 (no hardware motion) -----------------------
class FakeRcLayer4:
    """
    Emulates RcLayer4.send(): prints the command and blocks briefly
    to simulate the DONE wait for timed actions (ARC/DRIVE).
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._open = False

    def open(self):
        self._open = True
        print("[STUB L4] open()")

    def close(self):
        print("[STUB L4] close()")
        self._open = False

    def send(self, name: str, *args, **kwargs) -> Dict[str, Any]:
        """
        Simulate the blocking behavior:
        - steering/center/stop: immediate
        - ARC/DRIVE: sleep up to ~0.35s to mimic DONE latency
        """
        t0 = time.time()
        name_norm = name.strip().lower()

        # Determine if this is a timed action and extract duration (if any)
        done_tag = None
        sleep_s = 0.0

        if name_norm in ("arcleft", "arcright"):
            done_tag = "ARC"
            # args: (pwm, dur_ms, end="U")
            dur_ms = int(args[1]) if len(args) >= 2 else 0
            sleep_s = min(dur_ms / 1000.0, 0.35)
        elif name_norm in ("driveforward", "driveback"):
            done_tag = "DRIVE"
            dur_ms = int(args[1]) if len(args) >= 2 else 0
            sleep_s = min(dur_ms / 1000.0, 0.35)
        else:
            done_tag = None

        # Pretty print
        print(f"[STUB L4] send: {name}  args={args}  kw={kwargs}")

        # Simulate blocking for DONE
        if sleep_s > 0:
            time.sleep(sleep_s)

        return {
            "ok": done_tag or "IMMEDIATE",
            "done": done_tag,
            "elapsed_s": time.time() - t0
        }


# ----------------------- LIVE TELEM callback -----------------------
_last_telem = 0.0
def _telem_cb(d: Dict[str, Any]):
    global _last_telem
    if not PRINT_TELEM: return
    now = time.time()
    if now - _last_telem >= 0.2:  # ~5 Hz throttle
        print(f"TELEM drive={d.get('drive')} steer={d.get('steer')} L={d.get('L')} R={d.get('R')} t={d.get('t')}")
        _last_telem = now


# ----------------------- Vision selection -----------------------
def start_vision(bus: VisionBus):
    """
    Start the vision source per VISION config.
    Returns: a thread-like object with .stop() and .join().
    """
    if VISION.upper() == "CAM":
        try:
            from vision_pinsight import PiNSightDepthAI
            cam = PiNSightDepthAI(
                bus,
                blob_path=BLOB_PATH,
                conf_threshold=CONF_THRESHOLD,
                fps=FPS,
                show_preview=SHOW_PREVIEW
            )

            cam.probe()   # <--- fail fast if blob/device not OK
            cam.start()
            print("[VISION] Using PiNSIGHT/DepthAI camera")
            return cam
        except Exception as e:
            print(f"[VISION] PiNSIGHT not available ({e}). Falling back to SyntheticVision.")
    # fallback or VISION="FAKE"
    synth = SyntheticVision(bus, period_s=0.05)
    synth.start()
    print("[VISION] Using SyntheticVision (FAKE)")
    return synth


# ----------------------- Main -----------------------
if __name__ == "__main__":
    # Vision feed (camera preferred)
    bus = VisionBus()
    vision_thread = start_vision(bus)

    # Choose L4 implementation
    if MODE.upper() == "LIVE":
        l4 = RcLayer4(port=PORT, telemetry_cb=_telem_cb if PRINT_TELEM else None)
    else:
        l4 = FakeRcLayer4()

    # Start L4 + L5
    l4.open()
    params = HFParams(
        verbose=True,

        # --- calm the left/right so it can go forward ---
        deadband_x=0.16,     # wider “no-steer” window (was 0.08)
        k_arc_pwm=700,       # softer steering (was 900)
        arc_impulse_ms=180,  # shorter steering nudges (was 220)
        cooldown_ms=90,      # fewer back-to-back turns (was 60)

        # --- your follow-distance tuning ---
        target_area=TARGET_AREA,
        area_tol=AREA_TOL,
        k_drive_pwm=DRIVE_K,
        max_drive_pwm=MAX_DRIVE_PWM,
        drive_impulse_ms=DRIVE_MS,
    )
 # set verbose=False if you want quieter logs
    l5 = Layer5(l4, bus, params)
    l5.start()

    # Let it run for a while (or until Ctrl+C)
    try:
        t0 = time.time()
        while RUN_SECONDS <= 0 or (time.time() - t0 < RUN_SECONDS):
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Graceful shutdown
        l5.stop()
        vision_thread.stop()
        l5.join(timeout=1.0)
        try:
            vision_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            l4.close()
        except Exception:
            pass

        print("\n[L5 CHECK COMPLETE]\n")

"""
WHAT TO EXPECT

STUB + CAM:
- Real camera detections (person) drive the Layer-5 decisions, but FakeRcLayer4 logs commands only.
- Console: [INIT] → [SEARCH]/[ACQUIRE]/[FOLLOW] with ARC/DRIVE messages.
- No motor movement.

LIVE + CAM:
- Same decisions, but RcLayer4 talks to the Teensy. You'll see TELEM lines at ~5 Hz if PRINT_TELEM=True.
- Keep the robot on a stand for first runs.

If CAM isn't available (DepthAI/model missing), you'll see a clear message and the script uses SyntheticVision instead.
"""
