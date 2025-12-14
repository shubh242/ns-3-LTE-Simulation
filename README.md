# LTE A3-Based Handover Simulation (ns-3.39)

This project simulates 4G LTE handover behavior using Event A3-based RSRP measurements in the ns-3.39 simulator. It evaluates the effects of Time-to-Trigger (TTT), Hysteresis (Hys), and UE speed on handover rates and performance.

## Key Features

- 21-cell (7-site × 3-sector) hexagonal LTE topology
- RSRP-based A3 handover algorithm
- Configurable UE speeds and directions
- Packet-level simulation with UDP traffic
- KPIs: handover rate, ping-pong rate, throughput, jitter, delay
- Visual demo via NetAnim
- Auto-generated CSV + plots

## Requirements

- OS: macOS or Linux
- C++17 compiler
- Python 3.9+
- ns-3.39
- Python dependencies:
  ```bash
  pip install pandas matplotlib lxml
  ```

## Repository Structure

```
ns-3.39/
├── scratch/
│   └── a3_hex_21cells_control_only.cc      # Main simulation
├── scripts/
│   ├── plot_ho_rate_bar.py
│   ├── plot_pingpong_bar.py
│   ├── plot_timeseries.py
│   └── plot_flow_metrics.py
├── results/                                # Simulation outputs
├── plots/                                  # Graphs will be saved here
└── README.md
```

## Setup and Build

### Step 1: Clone and build ns-3

```bash
./build.py --enable-examples --enable-tests
cd ns-3.39
```

### Step 2: Build the custom simulation

```bash
./ns3 build
```

## Run Simulations

### Single run example:

```bash
./ns3 run "a3_hex_21cells_control_only \
  --speedKmph=30 --ttt=200 --hyst=3 --simTime=25"
```

This will output:
- `results/ho_summary.csv`
- `results/ho_events.csv`
- `results/handover_timeseries.csv`

### Parameter sweep example:

```bash
for SPEED in 3 30; do
  for H in 1 3 5; do
    for TTT in 80 160 320; do
      ./ns3 run "a3_hex_21cells_control_only \
        --speedKmph=$SPEED --hyst=$H --ttt=$TTT --simTime=25"
    done
  done
done
```

## Generate Plots

### Handover rate

```bash
python3 scripts/plot_ho_rate_bar.py \
  --csv results/ho_summary.csv \
  --speed 8.33 \
  --out plots/ho_rate_speed_8_33.png
```

### Ping-pong rate

```bash
python3 scripts/plot_pingpong_bar.py \
  --events results/ho_events.csv \
  --out plots/pingpong_rate.png
```

### Throughput timeseries

```bash
python3 scripts/plot_timeseries.py \
  --csv results/handover_timeseries.csv \
  --out plots/handover_timeseries.png
```

### Flow metrics (loss, delay, jitter)

```bash
python3 scripts/plot_flow_metrics.py \
  --xml results/handover_flow.xml \
  --out-prefix plots/handover
```

## NetAnim Visualization

```bash
NetAnim results/handover_demo.xml
```

## Notes

- Some parameters like `Tf` and `Tm` are accepted but **not used** in ns-3.39.
- Simulation logs HO events, KPIs, and packet stats to stdout and CSVs.
- Make sure `fading_trace_ETU_3kmph.fad` is present (not excluded by `.gitignore`).

