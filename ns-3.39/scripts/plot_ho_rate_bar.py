#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os

ap = argparse.ArgumentParser()
ap.add_argument("--csv", default="ho_summary.csv")
ap.add_argument("--speed", type=float, required=True)
ap.add_argument("--out", required=True)
a = ap.parse_args()

# Read CSV with header
df = pd.read_csv(a.csv)

# Ensure numeric columns (coerce errors -> NaN)
num_cols = [
    "speed_mps",
    "hysteresis_db",
    "ttt_ms",
    "num_ues",
    "sim_time_s",
    "ho_total",
    "ho_per_user_per_sec"
]
for c in num_cols:
    df[c] = pd.to_numeric(df[c], errors="coerce")

# Drop bad rows
df = df.dropna(subset=["speed_mps", "ho_per_user_per_sec"])

# Filter by speed
d = df[df["speed_mps"].round(3) == round(a.speed, 3)].copy()
if d.empty:
    raise SystemExit(f"No rows found for speed {a.speed}")

# Label formatting
d["hyst_str"] = d["hysteresis_db"].astype(int).astype(str) + " dB"
d["ttt_str"]  = d["ttt_ms"].astype(int).astype(str) + " ms"

# Pivot table
pv = d.pivot_table(
    index="hyst_str",
    columns="ttt_str",
    values="ho_per_user_per_sec",
    aggfunc="mean"
)

# Force expected ordering
pv = pv.reindex(
    index=["1 dB", "3 dB", "5 dB"],
    columns=["80 ms", "160 ms", "320 ms"]
).fillna(0)

# Plot
fig, ax = plt.subplots(figsize=(8, 4))
y = range(len(pv.index))
w = 0.25

for i, col in enumerate(pv.columns):
    ax.barh([yy + (i - 1) * w for yy in y], pv[col], height=w, label=col)

ax.set_yticks(list(range(len(pv.index))))
ax.set_yticklabels(pv.index)
ax.set_xlabel("Handovers per user per second")
ax.set_ylabel("Hysteresis")
ax.legend(title="Time-to-trigger")
ax.grid(axis="x", alpha=0.3)
ax.set_title(f"HO rate at speed {a.speed:.3f} m/s")

fig.tight_layout()
os.makedirs(os.path.dirname(a.out), exist_ok=True)
fig.savefig(a.out, dpi=200)
print("Wrote", a.out)
