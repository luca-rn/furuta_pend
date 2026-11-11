"""
Swing test: compare the simulated pendulum with the real one.

With REAL_CSV set, it loads a recording made with capture_swing.py, starts the
simulation from the same release angle, and compares period and decay.
Without it, it just runs the simulation.

Usage:  python swing_test.py
"""

import time

import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np

# ---- Settings ---------------------------------------------------------------
MODEL_PATH = "furuta_pend.xml"
REAL_CSV = "swing45.csv"  # recording from capture_swing.py, or None
RAW0 = None                  # raw AS5600 value with the pendulum hanging still.
                             # None = estimate it from the end of the recording.
START_ANGLE_DEG = 10.0       # only used when REAL_CSV is None
DURATION = 20.0              # seconds to simulate (capped at the recording length)
SHOW_VIEWER = False          # True: watch the sim in real time
MIN_AMP_DEG = 1.0            # ignore swings smaller than this (sensor noise)


# ---- Simulation -------------------------------------------------------------
def simulate(start_deg, duration, show_viewer):
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    data.joint("pend").qpos[0] = np.radians(start_deg)
    data.ctrl[:] = 0.0  # position actuator holds the arm at 0

    times, angles = [], []

    def step_and_log():
        t_before = data.time
        mujoco.mj_step(model, data)
        if data.time < t_before:
            raise SystemExit(
                "Simulation went unstable and MuJoCo reset it (check the terminal "
                "for a 'Nan, Inf or huge value' warning). Fix the model and rerun.")
        times.append(data.time)
        angles.append(np.degrees(data.joint("pend").qpos[0]))

    if show_viewer:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            wall_start = time.perf_counter()
            while viewer.is_running() and data.time < duration:
                step_and_log()
                viewer.sync()
                lag = data.time - (time.perf_counter() - wall_start)
                if lag > 0:
                    time.sleep(lag)
    else:
        while data.time < duration:
            step_and_log()

    return np.array(times), np.array(angles)


# ---- Real data --------------------------------------------------------------
def load_real(path):
    d = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.int64)
    us, raw = d[:, 0], d[:, 1]

    # Time in seconds, robust to micros() rolling over
    t = np.concatenate([[0], np.cumsum(np.diff(us) % 2**32)]) / 1e6

    # Make the raw angle continuous across the 4095 -> 0 rollover
    step = (np.diff(raw) + 2048) % 4096 - 2048
    counts = np.concatenate([[0], np.cumsum(step)]).astype(float)

    if RAW0 is not None:
        first = (raw[0] - RAW0 + 2048) % 4096 - 2048
        counts = counts + first
    else:
        # Assume it has nearly stopped by the end: the median of the last
        # 5 s is the hanging position
        counts = counts - np.median(counts[t > t[-1] - 5.0])

    angle = counts * 360.0 / 4096.0

    # Start at the release: the largest deflection is where it was let go
    i0 = int(np.argmax(np.abs(angle)))
    return t[i0:] - t[i0], angle[i0:]


# ---- Analysis ---------------------------------------------------------------
def analyze(t, a):
    """Period and amplitude of each full swing, between upward zero crossings."""
    idx = np.where((a[:-1] < 0) & (a[1:] >= 0))[0]
    cross_t = t[idx] + (t[idx + 1] - t[idx]) * (-a[idx]) / (a[idx + 1] - a[idx])

    # Amplitude of each swing = half of (max - min) between crossings.
    # Robust to a small zero-offset error.
    amps = np.array([(a[i:j].max() - a[i:j].min()) / 2
                     for i, j in zip(idx[:-1], idx[1:])])
    periods = np.diff(cross_t)

    # Drop swings too small to measure reliably
    small = np.where(amps < MIN_AMP_DEG)[0]
    n = small[0] if len(small) else len(amps)
    amps, periods, cross_t = amps[:n], periods[:n], cross_t[:n]

    if n == 0:
        raise SystemExit("No full swings found. Check the data / zero offset.")

    half = np.where(amps < amps[0] / 2)[0]
    return {
        "period": periods.mean(),
        "period_std": periods.std(),
        "n": n,
        "amps": amps,
        "swings_to_half": int(half[0]) if len(half) else None,
    }


def describe(name, r):
    half = (f"{r['swings_to_half']} swings" if r["swings_to_half"] is not None
            else "did not halve (record or simulate longer)")
    print(f"{name:5s} period {r['period']:.4f} s "
          f"(+/- {r['period_std'] * 1000:.1f} ms over {r['n']} swings), "
          f"half amplitude after: {half}")


# ---- Main -------------------------------------------------------------------
if REAL_CSV:
    t_real, a_real = load_real(REAL_CSV)
    start = a_real[0]
    duration = min(DURATION, t_real[-1])
    print(f"Real release angle: {start:.2f} deg")
else:
    t_real = a_real = None
    start, duration = START_ANGLE_DEG, DURATION

t_sim, a_sim = simulate(start, duration, SHOW_VIEWER)
np.savetxt("sim_swing45.csv", np.column_stack([t_sim, a_sim]),
           delimiter=",", header="t,angle_deg", comments="")
sim = analyze(t_sim, a_sim)
describe("Sim", sim)

if t_real is not None:
    real = analyze(t_real, a_real)
    describe("Real", real)

    err = (sim["period"] - real["period"]) / real["period"] * 100
    print(f"\nPeriod difference: sim is {err:+.1f}% vs real")
    if abs(err) <= 3:
        print("  Within 3%. Good.")
    elif err < 0:
        print("  Sim swings too FAST: model mass/inertia too close to the pivot.")
    else:
        print("  Sim swings too SLOW: model mass/inertia too far from the pivot.")

    sh, rh = sim["swings_to_half"], real["swings_to_half"]
    if sh is None or (rh is not None and sh > rh):
        print("  Sim dies down too slowly -> increase pend damping.")
    elif rh is None or sh < rh:
        print("  Sim dies down too quickly -> decrease pend damping.")
    else:
        print("  Decay matches.")

# ---- Plot -------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

ax1.plot(t_sim, a_sim, lw=1, label="sim")
if t_real is not None:
    ax1.plot(t_real, a_real, lw=1, alpha=0.8, label="real")
    ax1.set_xlim(0, duration)
ax1.axhline(0, color="k", lw=0.5)
ax1.set_xlabel("time since release [s]")
ax1.set_ylabel("pendulum angle [deg]")
ax1.legend()
ax1.grid(True)

ax2.plot(sim["amps"], "o-", ms=3, label="sim")
if t_real is not None:
    ax2.plot(real["amps"], "o-", ms=3, label="real")
ax2.set_xlabel("swing number")
ax2.set_ylabel("amplitude [deg]")
ax2.set_title("Decay (curves should overlap once damping is tuned)")
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()