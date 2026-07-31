# SenseEdge PCBA — bring-up and test protocol

Ordered procedure to take a freshly assembled SenseEdge board from
bare-board to a verified, deployed fault-classification node. Do the stages
in order; do not proceed past a failed gate. Record every measurement in the
results table at the end — those numbers feed the paper.

Prerequisites: bench PSU with current limit, DMM, oscilloscope (≥100 MHz),
USB-C cable, the ESP32 companion flashed with the UART bridge firmware, the
management-core firmware image on the W25Q32 flash, and
`ml/senseedge_weights.hex` (signed-int8 domain — see `ml/FIX_signed_input.md`).

---

## Stage 0 — bare-board inspection (before any power)
- [ ] Visual/AOI: no bridges on the QFN-64, LDOs, ADC, accelerometer.
- [ ] DMM continuity: 3V3, 1V8, GND planes not shorted to each other.
- [ ] Confirm **Y1 is the active 20 MHz oscillator** (`ECS-2520S33-200`),
      **not** a 2-pin crystal (see `PCB_AUDIT.md`, MUST-FIX #1).
- [ ] Confirm the assembled accelerometer part and note its sensitivity.

## Stage 1 — power rails (no ASIC activity)
- [ ] Apply USB-C 5 V through a current-limited supply (limit 200 mA).
- [ ] Measure **3.30 V ± 5 %** at the AP2112K-3.3 output and Caravel
      vdda/vddio/analog pins.
- [ ] Measure **1.80 V ± 5 %** at the AP2112K-1.8 output and Caravel vccd.
- [ ] Record quiescent current. Expected: a few mA (LDO + leakage) before
      the clock runs. **Gate:** abort if either rail is out of range or
      inrush trips the limit.

## Stage 2 — clock and reset
- [ ] Scope the `clock` pin: **clean 20 MHz** CMOS square wave, full 3.3 V
      swing. **Gate:** if absent, the oscillator is wrong/unpopulated (MUST-FIX #1).
- [ ] Press reset (SW1): confirm RESETB goes low then returns to 3.3 V with
      the RC time constant.
- [ ] Confirm the management SPI flash (W25Q32) shows CLK/CS activity within
      a few ms of reset release (the core fetching boot code).

## Stage 3 — management-core boot and bus
- [ ] Via the ESP32 UART bridge, confirm the boot banner / firmware heartbeat.
- [ ] Configure GPIO 0–7 and 30–37 to **user-project mode** (the user macro's
      `io_oeb` only takes effect after this — firmware step, not silicon).
- [ ] Wishbone read of the STATUS register (offset `0x04`) returns a sane FSM
      state. **Gate:** no Wishbone ack ⇒ stop and debug clock/reset/config.

## Stage 4 — accelerator functional (known vectors)
- [ ] Load `ml/senseedge_weights.hex` into the NN weight region over Wishbone.
- [ ] Drive a **known feature vector** (use the vectors from
      `ml/train_senseedge_fixed.py::hw_infer`) directly through the register
      interface / LA path.
- [ ] Read `CLASS_RESULT` (offset `0x08`): `class_id` and `confidence` must
      match the `hw_infer()` golden **bit-exactly**. **Gate:** mismatch ⇒
      wrong weights or wrong input domain (re-check the signed-int8 fix).
- [ ] Read back the FFT bins (`FFT_DATA`, `0x10`) for an injected tone and
      confirm the peak lands in the expected bin.

## Stage 5 — full acquisition chain (real signal)
- [ ] Apply a known vibration/tone to the accelerometer (shaker or tap test).
- [ ] Confirm the SPI ADC (MCP3201) returns changing samples via the LA
      `sample_count`/`samples_valid` probes.
- [ ] Run the full pipeline; confirm one acquisition→alarm pass completes
      (LA `fft_done`→`nn_done`) and the STATUS/CLASS registers update.
- [ ] Verify STATUS LED and ALARM GPIO behave (LED on when healthy, alarm
      asserts on a fault-like input).

## Stage 6 — electrical characterization (paper data)
- [ ] **f_max shmoo:** sweep clock 5→25 MHz at nominal V; record max stable.
- [ ] **Energy/inference:** measure core current during one inference at
      nominal V and at the minimum-energy point; ×V×t → J/inference.
- [ ] **Per-block power:** toggle FFT/NN enables; attribute current to
      acquisition / FFT / features / NN / idle.
- [ ] **V_DD scaling:** repeat energy/inference across the vccd range;
      find the minimum-energy point.
- [ ] **Node power:** whole-board average at 1, 10, and continuous
      classifications/s with duty cycling; project battery life.

## Stage 7 — deployment validation (paper data)
- [ ] Mount on the motor testbed; inject the four physical fault conditions
      (healthy, bearing wear, imbalance, misalignment).
- [ ] Record a **live confusion matrix** (target: recover the ~90 % the
      hardware-exact model predicts once retrained on real features).
- [ ] Sweep the alarm consecutive-fault depth; record false-alarm rate.
- [ ] Run ≥ 2 weeks continuously; log drift.
- [ ] Perform one **in-field weight update** (retrain → Wishbone reload →
      accuracy delta) — demonstrates the field-updateable model.

---

## Results log (fill during test)

| Item | Spec / expected | Measured | Pass |
|---|---|---|---|
| 3V3 rail | 3.30 V ± 5 % | | |
| 1V8 rail | 1.80 V ± 5 % | | |
| Quiescent current | few mA pre-clock | | |
| Clock | 20 MHz clean CMOS | | |
| Boot | flash activity post-reset | | |
| Wishbone STATUS | valid FSM state | | |
| Known-vector class | bit-exact vs golden | | |
| FFT tone bin | correct bin | | |
| f_max | ≥ 20 MHz | | |
| Energy/inference @ nominal | (record) | | |
| Energy/inference @ MEP | (record) | | |
| Avg node power @ 1/s | (record) | | |
| Testbed accuracy | ~90 % (post-retrain) | | |
| False-alarm rate | (record) vs filter depth | | |
