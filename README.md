# Traffic Light Controller (Verilog)

A Moore FSM-based traffic light controller for a 2-road intersection, with pedestrian crossing signals and request latches. Designed and debugged from scratch as a progressive learning project, following a control-path / datapath split.

## Features
- 4-state FSM (S0–S3) controlling NS/EW traffic lights (Green → Yellow → Red cycle)
- Parameterized datapath counter for state timing
- Pedestrian Walk / Flashing / Don't-Walk signals derived from FSM state
- Pedestrian pushbutton request latches (set-reset logic with priority ordering)
- Fully simulated and verified with Icarus Verilog + GTKWave

## State Table

| State | NS Light | EW Light | NS Pedestrian | EW Pedestrian |
|-------|----------|----------|----------------|----------------|
| S0    | Green    | Red      | Don't Walk     | Walk           |
| S1    | Yellow   | Red      | Flashing       | Walk           |
| S2    | Red      | Green    | Walk           | Flashing       |
| S3    | Red      | Yellow   | Walk           | Don't Walk     |

## Architecture
- **Traffic_Data** — datapath: parameterized down-counter, generates `timer_done`
- **Traffic_Control** — control path: FSM, Ld_G/Ld_Y pulse generation, light outputs, pedestrian request latches
- **Traffic** — top-level module wiring datapath + control path together
- **Traffic_TB** — testbench with clock generation, reset sequencing, and button stimulus

## How to Run
```
iverilog -o Traffic_Cntrl Traffic.v Traffic_TB.v
vvp Traffic_Cntrl
gtkwave Traffic.vcd
```

## Key Design Notes
- Ld_G/Ld_Y are generated as one-cycle pulses (via a `prv_state` edge-detect) rather than held high, to avoid continuously reloading the datapath counter.
- Pedestrian request latches use a fixed priority order: reset > button press (set) > Walk-phase-start (clear).
- Outputs are Moore-style (depend only on `state`), keeping timing and output logic cleanly separated.
