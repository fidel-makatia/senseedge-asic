# SPDX-License-Identifier: Apache-2.0
# SenseEdge - Caravel co-simulation 2: alarm across windows and a weight update that flips
# the class, through the real management core.  See senseedge_wb2.c for the phase protocol.
import math
import cocotb
from cocotb.triggers import ClockCycles, RisingEdge
from caravel_cocotb.caravel_interfaces import test_configure, report_test

REC8 = [2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694] * 8                       # tone at bin 8
REC2 = [int(round(2048 + 500 * math.sin(2 * math.pi * 2 * n / 64))) for n in range(64)]  # tone at bin 2
state = {"rec": REC8}


async def mcp3201(caravelEnv):
    idx, fall, cur, prev_cs, prev_sclk = 0, 0, 0, 1, 0
    caravelEnv.drive_gpio_in(0, 0)
    while True:
        await RisingEdge(caravelEnv.clk)
        g2, g1 = caravelEnv.monitor_gpio(2), caravelEnv.monitor_gpio(1)
        cs = int(g2.binstr, 2) if g2.is_resolvable else 1
        sclk = int(g1.binstr, 2) if g1.is_resolvable else 0
        if prev_cs == 1 and cs == 0:
            cur = state["rec"][idx % 64]; idx += 1; fall = 0
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
async def senseedge_wb2(dut):
    caravelEnv = await test_configure(dut, timeout_cycles=3_000_000)
    cocotb.log.info("[TEST] SenseEdge co-simulation 2: alarm, clear, weight update")
    await caravelEnv.release_csb()
    cocotb.start_soon(mcp3201(caravelEnv))

    await caravelEnv.wait_mgmt_gpio(1); cocotb.log.info("[TEST] firmware running")
    await caravelEnv.wait_mgmt_gpio(0); cocotb.log.info("[TEST] weights A verified; four fault windows follow")
    await caravelEnv.wait_mgmt_gpio(1)
    alarm = int(caravelEnv.monitor_gpio(3).binstr, 2)
    cocotb.log.info(f"[TEST] firmware saw class 1 x4 and the alarm after the 4th; alarm pad = {alarm}")
    assert alarm == 1, "alarm pad should be high"
    state["rec"] = REC2
    await caravelEnv.wait_mgmt_gpio(0)
    alarm = int(caravelEnv.monitor_gpio(3).binstr, 2)
    cocotb.log.info(f"[TEST] healthy windows classified 0, alarm cleared, weights B verified; alarm pad = {alarm}")
    assert alarm == 0, "alarm pad should be low"
    state["rec"] = REC8
    await caravelEnv.wait_mgmt_gpio(1)
    await ClockCycles(caravelEnv.clk, 2000)
    if caravelEnv.monitor_mgmt_gpio() != "1":
        cocotb.log.error("[TEST] FAIL: firmware reports a mismatch (management GPIO toggling)")
        assert False
    cocotb.log.info("[TEST] PASS: same record classified 2 after the register-map weight update")
