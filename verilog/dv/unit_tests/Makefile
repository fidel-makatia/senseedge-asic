# SPDX-License-Identifier: Apache-2.0
# SenseEdge Unit Test Makefile
# Run with: make all  (runs all tests)
#           make tb_fft_engine  (runs single test)

IVERILOG = iverilog
VVP = vvp
RTL_DIR = ../../rtl

# RTL source files
RTL_SRCS = \
	$(RTL_DIR)/spi_adc_if.v \
	$(RTL_DIR)/fft_engine.v \
	$(RTL_DIR)/feature_extract.v \
	$(RTL_DIR)/nn_engine.v \
	$(RTL_DIR)/wb_interface.v \
	$(RTL_DIR)/alarm_logic.v \
	$(RTL_DIR)/senseedge_top.v

# Testbenches
TESTS = \
	tb_spi_adc_if \
	tb_fft_engine \
	tb_feature_extract \
	tb_nn_engine \
	tb_alarm_logic \
	tb_wb_interface \
	tb_senseedge_top

.PHONY: all clean $(TESTS)

all: $(TESTS)
	@echo ""
	@echo "========================================"
	@echo "  All unit tests completed"
	@echo "========================================"

tb_spi_adc_if: tb_spi_adc_if.v $(RTL_DIR)/spi_adc_if.v
	@echo ""
	@echo "--- Running: $@ ---"
	$(IVERILOG) -o $@.vvp $< $(RTL_DIR)/spi_adc_if.v
	$(VVP) $@.vvp

tb_fft_engine: tb_fft_engine.v $(RTL_DIR)/fft_engine.v
	@echo ""
	@echo "--- Running: $@ ---"
	$(IVERILOG) -o $@.vvp $< $(RTL_DIR)/fft_engine.v
	$(VVP) $@.vvp

tb_feature_extract: tb_feature_extract.v $(RTL_DIR)/feature_extract.v
	@echo ""
	@echo "--- Running: $@ ---"
	$(IVERILOG) -o $@.vvp $< $(RTL_DIR)/feature_extract.v
	$(VVP) $@.vvp

tb_nn_engine: tb_nn_engine.v $(RTL_DIR)/nn_engine.v
	@echo ""
	@echo "--- Running: $@ ---"
	$(IVERILOG) -o $@.vvp $< $(RTL_DIR)/nn_engine.v
	$(VVP) $@.vvp

tb_alarm_logic: tb_alarm_logic.v $(RTL_DIR)/alarm_logic.v
	@echo ""
	@echo "--- Running: $@ ---"
	$(IVERILOG) -o $@.vvp $< $(RTL_DIR)/alarm_logic.v
	$(VVP) $@.vvp

tb_wb_interface: tb_wb_interface.v $(RTL_DIR)/wb_interface.v
	@echo ""
	@echo "--- Running: $@ ---"
	$(IVERILOG) -o $@.vvp $< $(RTL_DIR)/wb_interface.v
	$(VVP) $@.vvp

tb_senseedge_top: tb_senseedge_top.v $(RTL_SRCS)
	@echo ""
	@echo "--- Running: $@ ---"
	$(IVERILOG) -o $@.vvp $< $(RTL_SRCS)
	$(VVP) $@.vvp

clean:
	rm -f *.vvp *.vcd
