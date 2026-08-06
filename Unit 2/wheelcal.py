# wheelcal.py  v0.2.2
"""
WHEEL TRIM CALIBRATION

The two drive servos are never identical, so the robot curves when told to
drive straight. This measures the difference and corrects it automatically.

HOW TO USE

  STEP 1  Set TUNE_MODE = True and run. Aim at the target and adjust
          THRESHOLD until "detected" stays near 100%. The script prints the
          colour it is seeing and suggests a threshold to use.

  STEP 2  Set TUNE_MODE = False and run again. The robot drives short legs,
          correcting itself each round until it goes straight, then saves the
          result to wheeltrim.txt. servos.py loads that file at startup, so
          there is nothing to copy across by hand.

SETUP

  - Target flat on the floor, roughly 15 cm in front of the robot.
  - Robot on the floor, wheels down, about 50 cm clear BEHIND it.
    It reverses during the test.
  - Battery on.
  - Ctrl-C stops it safely at any point.
"""

from servos import *
from camera import *
import time

# ============================ USER SETTINGS ============================

TUNE_MODE = False          # True = tune the threshold, False = calibrate wheels

# False: ignore any saved trim and measure the wheels from scratch.
# True : start from the saved trim and refine it. The result is only
#        saved if it is actually better than what was already there.
LOAD_EXISTING = False

# Target colour.  (L min, L max, A min, A max, B min, B max)
# Keep L loose and let A/B do the work - that survives lighting and distance.
THRESHOLD = (33, 42, -24, -7, 2, 25)  # green. Check it in TUNE_MODE.

GAIN      = 30            # camera gain_db

# --- calibration parameters (rarely need changing) ---
PAN_KP      = 0.10        # pan tracking gain. Do not exceed 0.25.
PAN_MIN_STEP = 0.4        # deg. Corrections smaller than this are held
                          # back and added up, so the mirror sits still
                          # between moves instead of being nudged every
                          # frame. A mirror that twitches during the
                          # 21 ms exposure smears the target and the
                          # bounding box goes to pieces.
FACE_TURN   = 0.08        # pivot rate while lining up on the target
TURN_PULSE_S = 0.15       # length of each turn burst
TURN_REST_S  = 0.40       # pause after each burst, so the pan can catch up
FACE_TOL    = 1.5         # deg of pan counted as "facing it"
FACE_MAX_S  = 8.0         # give up lining up after this long
TEST_SPEED  = 0.35        # speed for each leg (driven in reverse)
PROBE       = 0.10        # trim probe magnitude
REPEATS     = 3           # legs averaged per round
MAX_ROUNDS  = 7           # give up if it has not converged by now
TOL_DEG     = 2.0         # drift counted as straight enough
TRIM_LIMIT  = 0.5         # never accept a trim beyond this
TRIM_FILE   = "wheeltrim.txt"   # saved here; servos.py reads it
SETTLE_S    = 0.8         # let the pan settle either side of a leg
MAX_PROBE_S = 4.0         # longest reverse while finding the safe leg length
LOST_ABORT  = 6           # frames without the target before a leg is stopped
BLIND_ABORT = 40          # frames without the target before returning gives up
RECOVER_FWD_S = 2.0       # forward run when trying to re-find the target
SCAN_LIMIT  = 30          # deg either side for the pan scan
SCAN_STEP   = 4           # deg between scan positions
SCAN_DWELL_MS = 120       # look time at each scan position
WIDTH_FRAC  = 0.55        # backstop: leg ends if the blob shrinks this far
CY_MARGIN   = 10          # px from the top of frame counted as out of range
CY_TOL      = 5           # px tolerance when restoring distance
LEG_FRAC    = 0.70        # use this much of the safe distance for real legs

CENTRE_PAUSE_S = 5        # hold at pan 0 so the mirror can be aligned by hand

# =======================================================================


def val(x):
    """Statistics are attributes on some builds, methods on others."""
    return x() if callable(x) else x


servo = Servo()
servo.soft_reset()

# servos.py loads any saved trim at startup. For a from-scratch calibration we
# must drop it, otherwise round 0 measures the residual rather than the wheels.
_saved_trim = servo.drive_trim
if LOAD_EXISTING:
    print("refining from saved trim %+.4f" % _saved_trim)
else:
    servo.drive_trim = 0.0
    if _saved_trim != 0.0:
        print("ignoring saved trim %+.4f - measuring from scratch" % _saved_trim)
cam = Cam([THRESHOLD], GAIN)          # exactly one threshold - one target


def centre_and_wait():
    servo.set_angle(0)
    print("\n" + "=" * 56)
    print("  PAN AT 0 - align the mirror to point straight ahead")
    print("=" * 56)
    t_end = time.ticks_add(time.ticks_ms(), CENTRE_PAUSE_S * 1000)
    shown = None
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        cam.get_blobs(0)
        left = time.ticks_diff(t_end, time.ticks_ms()) // 1000 + 1
        if left != shown:
            print("  %d..." % left)
            shown = left
    servo.set_angle(0)
    print("")


# =======================================================================
#  STEP 1 - threshold tuning
# =======================================================================
if TUNE_MODE:
    centre_and_wait()
    print("THRESHOLD TUNING. Aim at the target. Ctrl-C when happy.")
    print("Adjust THRESHOLD until 'detected' stays near 100%.")
    print("Also note the width - you want a good solid blob.\n")

    seen = total = fsum = 0
    lo = [999.0, 999.0, 999.0]
    hi = [-999.0, -999.0, -999.0]
    wsum = 0

    while True:
        blobs, img, _ = cam.get_blobs(0)
        big = cam.get_biggest_blob(blobs)
        total += 1

        if big is not None:
            seen += 1
            wsum += big.w
            img.draw_rectangle(big.rect)

            # sample the middle of the blob, not the bounding box
            cw = max(4, big.w // 3)
            ch = max(4, big.h // 3)
            rx0 = max(0, min(sensor.width() - cw, big.cx - cw // 2))
            ry0 = max(0, min(sensor.height() - ch, big.cy - ch // 2))
            roi = (rx0, ry0, cw, ch)
            img.draw_rectangle(roi)

            st = img.get_statistics(roi=roi)
            vals = (val(st.l_mean), val(st.a_mean), val(st.b_mean))
            for i in range(3):
                lo[i] = min(lo[i], vals[i])
                hi[i] = max(hi[i], vals[i])
            fill = 100 * big.pixels // max(1, big.w * big.h)
            fsum += fill

        if total % 20 == 0:
            rate = 100.0 * seen / total
            if seen:
                print("detected %5.1f%%  width %3d px  fill %d%%   "
                      "centre L %.0f..%.0f  a %.0f..%.0f  b %.0f..%.0f"
                      % (rate, wsum // seen, fsum // seen, lo[0], hi[0],
                         lo[1], hi[1], lo[2], hi[2]))
                print("   suggested THRESHOLD = (%d, %d, %d, %d, %d, %d)"
                      % (max(0, lo[0] - 20), min(100, hi[0] + 20),
                         lo[1] - 20, hi[1] + 20, lo[2] - 20, hi[2] + 20))
                w = wsum // seen
                # how far the target can move before the blob clips an edge
                px_per_deg = sensor.width() / cam.h_fov
                head_px = 160 - w // 2
                head_deg = head_px / px_per_deg
                print("   headroom before clipping: %d px = %.1f deg"
                      % (head_px, head_deg))
                if head_deg < 3.0:
                    print("   TIGHT - move the target FURTHER AWAY.")
                    print("   Blob width goes as 1/distance, so backing")
                    print("   off 30%% takes %d px to about %d px."
                          % (w, int(w / 1.3)))
            else:
                print("detected %5.1f%%  nothing seen - widen THRESHOLD" % rate)
            seen = total = wsum = fsum = 0


# =======================================================================
#  STEP 2 - wheel trim calibration
# =======================================================================

# banked pan correction, applied once it exceeds PAN_MIN_STEP
_pend = [0.0]


def look():
    """One frame of pan tracking.

    Returns:
        (pan_pos, width, cy, top_edge), or None if the target is not seen.
        Larger cy means the target is closer.
    """
    blobs, img, rotated = cam.get_blobs(servo.pan_pos)
    big = cam.get_biggest_blob(blobs)
    if big is None:
        return None
    rx = rotated[blobs.index(big)][0]
    # small cross = raw blob centre; large cross = after the periscope roll
    # correction. They separate as the pan angle grows - that is expected.
    img.draw_rectangle(big.rect)
    img.draw_cross((int(big.cx), int(big.cy)), size=4)
    img.draw_cross((int(rx), int(big.cy)), size=12)
    angle_error = -((rx - cam.w_centre) / sensor.width() * cam.h_fov)

    # optical deg -> servo deg, integrated so the target is genuinely centred.
    # Small corrections are banked and applied once they add up, so the mirror
    # is not disturbed on every frame.
    _pend[0] += angle_error * PAN_KP / cam.mirror_lever
    if abs(_pend[0]) >= PAN_MIN_STEP:
        servo.set_angle(servo.pan_pos + _pend[0])
        _pend[0] = 0.0

    return servo.pan_pos, big.w, big.cy, big.cy - big.h // 2


def settle(secs):
    t_end = time.ticks_add(time.ticks_ms(), int(secs * 1000))
    last = None
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        r = look()
        if r is not None:
            last = r
    return last


# Which way to spin the body to reduce a given pan angle. Depends on wiring
# and mirror geometry, so it is DISCOVERED rather than assumed: the first
# attempt tries one way and flips if the pan angle grows instead of shrinking.
# Once found it is reused, so later legs line up immediately.
TURN_SIGN = [1]


def wait_target(timeout_s=3.0):
    """Block until the target is seen, or give up. Returns pan or None."""
    t_end = time.ticks_add(time.ticks_ms(), int(timeout_s * 1000))
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        r = look()
        if r is not None:
            return r[0]
    return None


def turn_pulse(signed_rate):
    """Turn the body in one short burst, then stop and let the pan catch up.

    Args:
        signed_rate (float): turn rate, sign sets the direction.

    Returns:
        float: settled pan angle, or None if the target was lost.
    """
    servo.set_drive(0, signed_rate)
    t_end = time.ticks_add(time.ticks_ms(), int(TURN_PULSE_S * 1000))
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        look()
    servo.set_drive(0, 0)

    # rest: no driving, pan alone converges on the target
    last = None
    t_end = time.ticks_add(time.ticks_ms(), int(TURN_REST_S * 1000))
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        r = look()
        if r is not None:
            last = r[0]
    return last


def probe_turn_sign():
    """Work out which way to spin the body to reduce the pan angle.

    Returns:
        bool: True if the direction was established.
    """
    p0 = wait_target()
    if p0 is None:
        print("  target not visible - cannot probe turn direction")
        return False

    d = 1 if p0 > 0 else -1
    print("  probing turn direction (pan starts at %+.1f deg)..." % p0)

    p1 = turn_pulse(TURN_SIGN[0] * FACE_TURN * d)

    if p1 is None or abs(p1) > abs(p0) + 0.5:
        TURN_SIGN[0] = -TURN_SIGN[0]
        why = "target lost" if p1 is None else "pan grew to %+.1f" % p1
        print("  %s -> turn sign flipped to %+d" % (why, TURN_SIGN[0]))
        turn_pulse(TURN_SIGN[0] * FACE_TURN * d)      # undo
    else:
        print("  pan improved to %+.1f -> turn sign %+d is correct"
              % (p1, TURN_SIGN[0]))
    return True


def face_target():
    """Pulse the body round until the pan is near zero (robot faces target)."""
    t_end = time.ticks_add(time.ticks_ms(), int(FACE_MAX_S * 1000))
    p = wait_target()
    if p is None:
        print("  target not visible")
        return False

    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        if abs(p) < FACE_TOL:
            servo.set_drive(0, 0)
            return True
        if abs(p) > 45:
            servo.set_drive(0, 0)
            print("  pan reached %.0f deg - target beyond reach" % p)
            return False

        nxt = turn_pulse(TURN_SIGN[0] * FACE_TURN * (1 if p > 0 else -1))
        if nxt is None:
            servo.set_drive(0, 0)
            print("  lost the target while lining up - stopping")
            return False
        p = nxt

    servo.set_drive(0, 0)
    print("  timed out lining up (pan still %.1f deg)" % p)
    return False


def reverse_for(secs, trim=0.0):
    """Reverse for a fixed time. Returns (pan_drift, ok)."""
    servo.drive_trim = trim
    start = settle(SETTLE_S)
    if start is None:
        return None, False
    # Stop the moment the target goes out of sight - never drive blind.
    servo.set_drive(-TEST_SPEED, 0)
    t_end = time.ticks_add(time.ticks_ms(), int(secs * 1000))
    lost = 0
    aborted = False
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        if look() is None:
            lost += 1
            if lost > LOST_ABORT:
                aborted = True
                break
        else:
            lost = 0
    servo.set_drive(0, 0)
    if aborted:
        print("    target lost mid-leg - stopped")
        return None, False
    end = settle(SETTLE_S)
    if end is None:
        return None, False
    return end[0] - start[0], True


def go_forward(secs):
    # keep whatever trim is set, so forward runs get corrected too
    servo.set_drive(TEST_SPEED, 0)
    time.sleep_ms(int(secs * 1000))
    servo.set_drive(0, 0)
    settle(SETTLE_S)


def restore_distance(cy0, timeout_s=5.0):
    """Drive forward until the target is back at its starting distance.

    Args:
        cy0 (int): image row to drive back to. Larger cy = closer.

    Returns:
        bool: True if the distance was restored.
    """
    # keep whatever trim is set, so forward runs get corrected too
    t_end = time.ticks_add(time.ticks_ms(), int(timeout_s * 1000))
    lost = 0
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        r = look()
        if r is None:
            # keep going briefly to re-acquire, but give up rather than run blind
            lost += 1
            if lost > BLIND_ABORT:
                servo.set_drive(0, 0)
                print("    (lost target while returning - stopped)")
                settle(SETTLE_S)
                return False
        else:
            lost = 0
            if r[2] >= cy0 - CY_TOL:
                servo.set_drive(0, 0)
                settle(SETTLE_S)
                return True
        servo.set_drive(TEST_SPEED, 0)
    servo.set_drive(0, 0)
    settle(SETTLE_S)
    print("    (could not fully restore distance)")
    return False


def reacquire():
    """Try to find a lost target: drive forward, then pan-scan, then give up.

    Returns:
        bool: True if the target was found again.
    """
    print("    lost target - driving forward to re-find it...")
    servo.drive_trim = 0.0
    servo.set_drive(TEST_SPEED, 0)
    t_end = time.ticks_add(time.ticks_ms(), int(RECOVER_FWD_S * 1000))
    found = False
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        if look() is not None:
            found = True
            break
    servo.set_drive(0, 0)
    if found:
        print("    found it")
        return True

    print("    still nothing - scanning with the pan...")
    a = -SCAN_LIMIT
    while a <= SCAN_LIMIT:
        servo.set_angle(a)
        t2 = time.ticks_add(time.ticks_ms(), SCAN_DWELL_MS)
        while time.ticks_diff(t2, time.ticks_ms()) > 0:
            blobs, img, _ = cam.get_blobs(0)
            _b = cam.get_biggest_blob(blobs)
            if _b is not None:
                img.draw_rectangle(_b.rect)
                print("    found at pan %+d deg" % a)
                settle(SETTLE_S)
                return True
        a += SCAN_STEP

    servo.set_angle(0)
    print("    scan found nothing")
    return False


centre_and_wait()
print("=" * 58)
print("  WHEEL TRIM CALIBRATION - THE ROBOT DRIVES BACKWARD")
print("  Clear ~50 cm BEHIND it. Ctrl-C stops it.")
print("=" * 58)
time.sleep_ms(3000)

if settle(1.5) is None:
    print("Target not visible. Run with TUNE_MODE = True first.")
    servo.set_drive(0, 0)
    servo.soft_reset()
    raise SystemExit

history = []
final_trim = None
final_drift = None
try:
    # ---- phase 0: how far can we reverse before losing it? ----
    print("\n--- phase 0: turn direction + safe leg length ---")
    if not probe_turn_sign():
        raise SystemExit
    if not face_target():
        if not reacquire() or not face_target():
            print("  could not face the target, and could not recover it.")
            print("  Check the target is in view and re-run.")
            raise SystemExit

    r = settle(SETTLE_S)
    w0, cy0 = r[1], r[2]
    print("  start: width %d px, cy %d (larger cy = closer)" % (w0, cy0))
    if w0 > 112:
        print("  WARNING: blob is %d%% of frame width - it will clip the edge"
              % (100 * w0 // 320))
        print("  once tracking moves, biasing the centroid. Use a smaller object.")

    servo.drive_trim = 0.0
    servo.set_drive(-TEST_SPEED, 0)
    t0 = time.ticks_ms()
    safe_s = MAX_PROBE_S
    while True:
        el = time.ticks_diff(time.ticks_ms(), t0) / 1000.0
        if el > MAX_PROBE_S:
            break
        r = look()
        # out of range: lost, at the top of frame, or too small
        if r is None or r[3] < CY_MARGIN or r[1] < w0 * WIDTH_FRAC:
            safe_s = el
            if r is not None:
                print("  edge reached: cy %d, top %d, width %d"
                      % (r[2], r[3], r[1]))
            break
    servo.set_drive(0, 0)
    print("  target faded after %.2f s of reversing" % safe_s)

    leg_s = max(0.3, safe_s * LEG_FRAC)
    print("  using %.2f s legs" % leg_s)
    print("  returning to start (cy %d)..." % cy0)
    restore_distance(cy0)

    # ---- three legs: trim 0, +PROBE, -PROBE ----
    # ---- self-tuning: drive, measure, correct, repeat ----

    def measure_at(trim):
        """Mean pan drift over REPEATS legs at this trim.

        Returns:
            float: mean drift, or None if the target could not be recovered.
        """
        ds = []
        for rep in range(REPEATS):
            if not face_target():
                if not reacquire() or not face_target():
                    return None

            d, ok = reverse_for(leg_s, trim)

            if not restore_distance(cy0):
                if not reacquire():
                    return None
                restore_distance(cy0)

            if ok:
                ds.append(d)
                print("      leg %d: %+.2f deg" % (rep, d))
            elif not reacquire():
                return None

        if not ds:
            return None
        avg = sum(ds) / len(ds)
        if len(ds) > 1:
            print("      %d legs, spread %.2f deg" % (len(ds), max(ds) - min(ds)))
        else:
            print("      only 1 usable leg - treat this round with caution")
        return avg

    print("\n" + "=" * 58)
    print("  SELF-TUNING - watch the drift shrink each round")
    print("=" * 58)

    trim_a = _saved_trim if LOAD_EXISTING else 0.0
    d_a = measure_at(trim_a)
    if d_a is None:
        print("  could not measure the first round - target lost and could")
        print("  not be recovered. Aborting.")
        raise SystemExit
    print("\nround 0:  trim %+.4f  ->  drift %+.2f deg" % (trim_a, d_a))
    history.append((trim_a, d_a))

    trim_b = trim_a + (PROBE if d_a < 0 else -PROBE)
    d_b = measure_at(trim_b)
    if d_b is None:
        print("  could not measure at the probe trim - target lost")
        print("  and could not be recovered. Aborting.")
        raise SystemExit
    print("round 1:  trim %+.4f  ->  drift %+.2f deg" % (trim_b, d_b))
    history.append((trim_b, d_b))

    for it in range(2, MAX_ROUNDS):
        if abs(d_b) <= TOL_DEG:
            print("\n  within tolerance (%.1f deg) - done" % TOL_DEG)
            break

        span = trim_b - trim_a
        if abs(span) < 1e-6 or abs(d_b - d_a) < 1e-6:
            print("\n  no measurable response to trim - stopping")
            break
        slope = (d_b - d_a) / span

        trim_c = trim_b - d_b / slope
        if trim_c > TRIM_LIMIT:
            trim_c = TRIM_LIMIT
        elif trim_c < -TRIM_LIMIT:
            trim_c = -TRIM_LIMIT

        d_c = measure_at(trim_c)
        if d_c is None:
            print("  round %d: target lost and not recovered - stopping" % it)
            break
        print("round %d:  trim %+.4f  ->  drift %+.2f deg" % (it, trim_c, d_c))
        history.append((trim_c, d_c))

        trim_a, d_a = trim_b, d_b
        trim_b, d_b = trim_c, d_c

    final_trim, final_drift = trim_b, d_b

finally:
    servo.drive_trim = 0.0
    servo.set_drive(0, 0)
    servo.soft_reset()

# ---------------------------- verdict ----------------------------
print("\n" + "=" * 58)
if final_trim is None or not history:
    print("  Self-tuning did not complete.")
else:
    start_drift = history[0][1]
    print("  drift before tuning : %+.2f deg" % start_drift)
    print("  drift after tuning  : %+.2f deg" % final_drift)
    if abs(start_drift) > 1e-6:
        print("  improvement         : %.0f%%"
              % (100.0 * (1.0 - abs(final_drift) / abs(start_drift))))
    print("  final drive_trim    : %+.4f" % final_trim)

    better = True
    if LOAD_EXISTING and abs(final_drift) >= abs(start_drift):
        better = False
        print("\n  NOT SAVED: %+.2f deg is no better than the %+.2f deg the"
              % (final_drift, start_drift))
        print("  saved trim already gave. Keeping %+.4f." % _saved_trim)

    if better and abs(final_drift) <= TOL_DEG:
        try:
            with open(TRIM_FILE, "w") as f:
                f.write("%.4f\n" % final_trim)
            # read it back, so a silent write failure cannot pass as success
            with open(TRIM_FILE) as f:
                back = float(f.read().strip())
            if abs(back - final_trim) < 1e-4:
                print("\n  SAVED and verified: %s holds %+.4f" % (TRIM_FILE, back))
                print("  servos.py reads this at startup - the robot will now")
                print("  apply it automatically. Nothing to paste.")
            else:
                print("\n  WARNING: wrote %+.4f but read back %+.4f"
                      % (final_trim, back))
                print("  Put it in servos.py by hand instead.")
        except (OSError, ValueError) as e:
            print("\n  could not save (%s)" % e)
            print("  put this in servos.py by hand:")
            print("      self.drive_trim = %+.4f" % final_trim)
    elif better:
        print("\n  NOT SAVED: drift is still above the %.1f deg tolerance." % TOL_DEG)
        print("  Most of it is probably not wheel imbalance - more likely the")
        print("  target moving in frame. Nothing was written.")
print("=" * 58)
