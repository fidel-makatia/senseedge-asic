# SPDX-License-Identifier: Apache-2.0
# SenseEdge - Caravel co-simulation of the unmodified production firmware (corrected RTL).
# The test plays the ADC on GPIO 0..2 and reads the hardware UART on pad 6 with the
# harness's monitor.  Expected lines for the bin-8 record under the shipped weights
# (firmware/nn_weights.h), as computed on the RTL by verilog/dv/audit (class 0, conf 255):
#   "SenseEdge v1.1 Online", "WEIGHTS verified, word errors: 0", "Monitoring vibration...",
#   then one "CLASS:HEALTHY CONF:255 ALARM:0" line per window.
import cocotb
from cocotb.triggers import RisingEdge
from caravel_cocotb.caravel_interfaces import test_configure, report_test, UART

RECORD = [2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694] * 8
WINDOWS = 3


async def mcp3201(caravelEnv):
    idx, fall, cur, prev_cs, prev_sclk = 0, 0, 0, 1, 0
    caravelEnv.drive_gpio_in(0, 0)
    while True:
        await RisingEdge(caravelEnv.clk)
        g2, g1 = caravelEnv.monitor_gpio(2), caravelEnv.monitor_gpio(1)
        cs = int(g2.binstr, 2) if g2.is_resolvable else 1
        sclk = int(g1.binstr, 2) if g1.is_resolvable else 0
        if prev_cs == 1 and cs == 0:
            cur = RECORD[idx % 64]; idx += 1; fall = 0
            caravelEnv.drive_gpio_in(0, 1)
        if cs == 0 and prev_sclk == 1 and sclk == 0:
            fall += 1
            if fall == 2:
                caravelEnv.drive_gpio_in(0, 0)
            elif 3 <= fall <= 14:
                caravelEnv.drive_gpio_in(0, (cur >> (14 - fall)) & 1)
            elif fall >= 15:
                caravelEnv.drive_gpio_in(0, (cur >> (fall - 14)) & 1)
        prev_cs, prev_sclk = cs, sclk


@cocotb.test()
@report_test
async def senseedge_fw(dut):
    caravelEnv = await test_configure(dut, timeout_cycles=6_000_000)
    cocotb.log.info("[TEST] production firmware on the Caravel management core, hardware UART on pad 6")
    await caravelEnv.release_csb()
    cocotb.start_soon(mcp3201(caravelEnv))
    uart = UART(caravelEnv)
    await caravelEnv.wait_mgmt_gpio(1); cocotb.log.info("[TEST] GPIO configured")

    lines = []
    while sum(1 for l in lines if l.startswith("CLASS:")) < WINDOWS:
        line = await uart.get_line()
        lines.append(line); cocotb.log.info(f"[TEST] UART: {line}")
    banner_ok = any(l.startswith("SenseEdge v1.1 Online") for l in lines)
    weights_ok = any(l == "WEIGHTS verified, word errors: 0" for l in lines)
    cls = [l for l in lines if l.startswith("CLASS:")][:WINDOWS]
    cls_ok = all(l == "CLASS:HEALTHY CONF:255 ALARM:0" for l in cls)
    cocotb.log.info(f"[TEST] banner {banner_ok}, weights {weights_ok}, class lines {cls}")
    assert banner_ok, "banner missing or wrong"
    assert weights_ok, "weight read-back reported errors"
    assert cls_ok, "class lines differ from the RTL expectation"
    cocotb.log.info(f"[TEST] PASS: production firmware ran {WINDOWS} windows; UART lines as expected")
