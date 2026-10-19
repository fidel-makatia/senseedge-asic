# SPDX-License-Identifier: Apache-2.0
# SenseEdge - Caravel co-simulation (corrected RTL, branch fix/audit-defects)
# The test side plays the MCP3201 on GPIO 0..2 with the data-sheet frame timing used by the
# audit harness; the firmware loads, verifies and reads back everything over the real
# Caravel Wishbone master and reports PASS on the management GPIO.
import cocotb
from cocotb.triggers import ClockCycles, RisingEdge
from caravel_cocotb.caravel_interfaces import test_configure, report_test

# tone at bin 8, amplitude 500, around mid-scale (same record as the integration test)
RECORD = [2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694] * 8


async def mcp3201(caravelEnv):
    """After CS falls DOUT is high-Z (read as 1); a null bit on the 2nd falling SCLK edge,
    then B11..B0 on edges 3..14, then the LSB-first repeat B1, B2, ... on edges 15.."""
    idx, fall, cur, prev_cs, prev_sclk = 0, 0, 0, 1, 0
    caravelEnv.drive_gpio_in(0, 0)
    while True:
        await RisingEdge(caravelEnv.clk)
        cs = int(caravelEnv.monitor_gpio(2).binstr, 2) if caravelEnv.monitor_gpio(2).is_resolvable else 1
        sclk = int(caravelEnv.monitor_gpio(1).binstr, 2) if caravelEnv.monitor_gpio(1).is_resolvable else 0
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
async def senseedge_wb(dut):
    caravelEnv = await test_configure(dut, timeout_cycles=3_000_000)
    cocotb.log.info("[TEST] SenseEdge co-simulation: corrected RTL through the Caravel management core")
    await caravelEnv.release_csb()
    cocotb.start_soon(mcp3201(caravelEnv))

    await caravelEnv.wait_mgmt_gpio(1)
    cocotb.log.info("[TEST] firmware running")
    await caravelEnv.wait_mgmt_gpio(0)
    cocotb.log.info("[TEST] 53 weight words written and read back equal; window running")
    await caravelEnv.wait_mgmt_gpio(1)
    # a steady 1 is PASS; a failing firmware toggles the GPIO forever
    await ClockCycles(caravelEnv.clk, 2000)
    if caravelEnv.monitor_mgmt_gpio() != "1":
        cocotb.log.error("[TEST] FAIL: firmware reports a mismatch (management GPIO toggling)")
        assert False
    alarm = int(caravelEnv.monitor_gpio(3).binstr, 2)
    cocotb.log.info(f"[TEST] PASS: bins, features and class (1, 255) match the RTL expectation; alarm pad = {alarm}")
