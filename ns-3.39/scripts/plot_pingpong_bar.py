#!/usr/bin/env python3
import pandas as pd, matplotlib.pyplot as plt, argparse, os
ap=argparse.ArgumentParser()
ap.add_argument("--events", default="ho_events.csv")
ap.add_argument("--window", type=float, default=5.0, help="ping-pong window (s)")
ap.add_argument("--out", default="plots/pingpong_rate.png")
a=ap.parse_args()

if not os.path.exists(a.events):
    print(f"SKIP: {a.events} not found (your C++ must log HO events to use this plot).")
    raise SystemExit(0)

df=pd.read_csv(a.events)
# Expect: time_s, imsi, from_cell, to_cell; optionally speed/hyst/ttt for grouping
req={"time_s","imsi","from_cell","to_cell"}
if not req.issubset(df.columns): raise SystemExit(f"{a.events} missing columns {req-df.columns}")
df=df.sort_values(["imsi","time_s"])

def ping_pong_rate(g):
    # count A->B->A within time window
    events = g[["time_s", "from_cell", "to_cell"]].to_numpy()
    n = 0
    pp = 0
    for i in range(len(events) - 2):
        t1, a1, b1 = events[i]
        t2, a2, b2 = events[i+1]
        t3, a3, b3 = events[i+2]
        if b1 == a2 and b2 == a3 and a1 == b3 and (t3 - t1) <= a.window:
            pp += 1
        n += 1
    ratio = 0 if n == 0 else pp / max(1, n)
    return pd.Series({"pingpong": ratio})

rates = df.groupby("imsi", as_index=False).apply(ping_pong_rate)
rate = rates["pingpong"].mean()

fig,ax=plt.subplots(figsize=(5,3))
ax.bar(["Ping-pong fraction"], [rate])
ax.set_ylim(0,1); ax.grid(axis="y",alpha=.3)
fig.tight_layout(); fig.savefig(a.out,dpi=200); print("Wrote",a.out)
