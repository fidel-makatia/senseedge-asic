# SPDX-License-Identifier: Apache-2.0
# SenseEdge - Cocotb Integration Test
# Tests the full Caravel + SenseEdge pipeline via Wishbone

from caravel_cocotb.caravel_interfaces import test_configure
from caravel_cocotb.caravel_interfaces import report_test
import cocotb
from cocotb.triggers import ClockCycles, Timer


@cocotb.test()
@report_test
async def senseedge_wb(dut):
    caravelEnv = await test_configure(dut, timeout_cycles=500000)

    cocotb.log.info("[TEST] Starting SenseEdge Wishbone integration test")

    # Wait for firmware to start (management GPIO goes high = config phase)
    await caravelEnv.release_csb()
    await caravelEnv.wait_mgmt_gpio(1)
    cocotb.log.info("[TEST] Firmware started configuration phase")

    # Wait for firmware to finish weight loading (GPIO goes low)
    await caravelEnv.wait_mgmt_gpio(0)
    cocotb.log.info("[TEST] Weights loaded, system enabling...")

    # Wait for firmware to signal completion (GPIO goes high again)
    await caravelEnv.wait_mgmt_gpio(1)
    cocotb.log.info("[TEST] Pipeline completed")

    # Read Logic Analyzer outputs to verify classification
    # la_data_out[1:0]  = class_id
    # la_data_out[9:2]  = confidence
    # la_data_out[10]   = alarm_active
    # la_data_out[11]   = fft_busy
    # la_data_out[12]   = nn_busy
    # la_data_out[13]   = fe_busy
    # la_data_out[14]   = fft_done
    # la_data_out[15]   = nn_done
    # la_data_out[22]   = samples_valid
    # la_data_out[23]   = enable

    await ClockCycles(caravelEnv.clk, 10)

    # Check that the system is enabled
    la_out = caravelEnv.monitor_la(23, 23)
    cocotb.log.info(f"[TEST] Enable bit (LA[23]): {la_out}")

    # Check classification result
    class_id = int(caravelEnv.monitor_la(1, 0).binstr, 2)
    confidence = int(caravelEnv.monitor_la(9, 2).binstr, 2)
    alarm = int(caravelEnv.monitor_la(10, 10).binstr, 2)
    fft_busy = int(caravelEnv.monitor_la(11, 11).binstr, 2)
    nn_busy = int(caravelEnv.monitor_la(12, 12).binstr, 2)

    cocotb.log.info(f"[TEST] Classification: class={class_id}, confidence={confidence}")
    cocotb.log.info(f"[TEST] Alarm active: {alarm}")
    cocotb.log.info(f"[TEST] FFT busy: {fft_busy}, NN busy: {nn_busy}")

    # Verify pipeline is not stuck (both should be idle after completion)
    if fft_busy == 0 and nn_busy == 0:
        cocotb.log.info("[TEST] PASS: Pipeline idle after completion")
    else:
        cocotb.log.error("[TEST] FAIL: Pipeline still busy after timeout")

    # Check GPIO outputs
    # io_out[3] = alarm, io_out[4] = status LED
    gpio_alarm = caravelEnv.monitor_gpio(3, 3)
    gpio_led = caravelEnv.monitor_gpio(4, 4)
    cocotb.log.info(f"[TEST] GPIO alarm={gpio_alarm}, LED={gpio_led}")

    cocotb.log.info("[TEST] SenseEdge integration test complete")
