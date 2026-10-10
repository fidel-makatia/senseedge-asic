#!/bin/sh
# Simulate the Zybo wrapper with an AXI-Lite master that runs the same self-test as sw/senseedge_zybo.c.
cd "$(dirname "$0")" && R=../../../verilog/rtl && \
iverilog -g2012 -o tb_axil.vvp tb_axil_senseedge.v ../rtl/axil_senseedge.v ../rtl/mcp3201_emu.v \
  $R/spi_adc_if.v $R/fft_engine.v $R/feature_extract.v $R/nn_engine.v $R/wb_interface.v $R/alarm_logic.v $R/senseedge_top.v && \
vvp tb_axil.vvp | tail -4; rm -f tb_axil.vvp
