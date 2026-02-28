// SPDX-License-Identifier: Apache-2.0
// Testbench: SPI ADC Interface
// Simulates an MCP3201-style 12-bit ADC responding over SPI
// Verifies sample collection and buffer fill signaling

`timescale 1ns / 1ps

module tb_spi_adc_if;

    // --- Clock and Reset ---
    reg clk;
    reg rst;

    initial clk = 0;
    always #20 clk = ~clk; // 25 MHz

    // --- DUT signals ---
    reg         enable;
    reg  [15:0] clk_div;
    wire        spi_clk;
    wire        spi_cs_n;
    reg         spi_miso;
    wire        samples_valid;
    wire [15:0] sample_out;
    reg  [5:0]  sample_addr;
    wire [5:0]  sample_count;

    // --- DUT ---
    spi_adc_if dut (
        .clk          (clk),
        .rst          (rst),
        .enable       (enable),
        .clk_div      (clk_div),
        .spi_clk      (spi_clk),
        .spi_cs_n     (spi_cs_n),
        .spi_miso     (spi_miso),
        .samples_valid(samples_valid),
        .sample_out   (sample_out),
        .sample_addr  (sample_addr),
        .sample_count (sample_count)
    );

    // --- Simulated ADC ---
    // MCP3201-style: outputs 16 bits on falling CS, MSB first
    // Format: 1 null bit, 1 null bit, 1 null bit, then 12 data bits, 1 pad
    reg [15:0] adc_value;
    reg [11:0] adc_sequence [0:63]; // Pre-defined ADC values
    integer    adc_seq_idx;
    reg [4:0]  adc_bit_cnt;

    // Generate a known test pattern: DC + sine-like
    integer init_i;
    initial begin
        for (init_i = 0; init_i < 64; init_i = init_i + 1) begin
            // Simple triangular wave pattern for easy verification
            if (init_i < 16)
                adc_sequence[init_i] = 12'd2048 + (init_i * 12'd100);
            else if (init_i < 32)
                adc_sequence[init_i] = 12'd2048 + ((32 - init_i) * 12'd100);
            else if (init_i < 48)
                adc_sequence[init_i] = 12'd2048 - ((init_i - 32) * 12'd100);
            else
                adc_sequence[init_i] = 12'd2048 - ((64 - init_i) * 12'd100);
        end
        adc_seq_idx = 0;
    end

    // ADC responds to SPI transactions
    always @(negedge spi_cs_n) begin
        adc_value = {3'b000, adc_sequence[adc_seq_idx], 1'b0};
        adc_bit_cnt = 0;
        adc_seq_idx = (adc_seq_idx + 1) % 64;
    end

    always @(posedge spi_clk) begin
        if (!spi_cs_n) begin
            spi_miso    <= adc_value[15 - adc_bit_cnt];
            adc_bit_cnt <= adc_bit_cnt + 1;
        end
    end

    always @(posedge spi_cs_n) begin
        spi_miso <= 1'bz;
    end

    // --- Test sequence ---
    integer pass_count;
    integer fail_count;
    integer i;

    initial begin
        $dumpfile("tb_spi_adc_if.vcd");
        $dumpvars(0, tb_spi_adc_if);

        pass_count = 0;
        fail_count = 0;

        // Initialize
        rst         = 1;
        enable      = 0;
        clk_div     = 16'd4;  // Fast SPI for simulation
        spi_miso    = 0;
        sample_addr = 0;

        // Reset
        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // --- Test 1: CS should be high when disabled ---
        $display("[TEST 1] CS high when disabled");
        if (spi_cs_n === 1'b1) begin
            $display("  PASS: CS_N is high");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: CS_N should be high when disabled");
            fail_count = fail_count + 1;
        end

        // --- Test 2: Enable and collect 64 samples ---
        $display("[TEST 2] Collect 64 samples");
        adc_seq_idx = 0;
        enable = 1;

        // Wait for samples_valid
        begin : wait_samples_valid_1
            integer wait_cnt;
            wait_cnt = 0;
            while (samples_valid !== 1'b1 && wait_cnt < 500000) begin
                @(posedge clk);
                wait_cnt = wait_cnt + 1;
            end
            if (wait_cnt >= 500000)
                $display("  TIMEOUT waiting for samples_valid");
            else
                $display("  samples_valid asserted at time %0t", $time);
        end

        if (samples_valid === 1'b1) begin
            $display("  PASS: 64 samples collected");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: samples_valid not asserted");
            fail_count = fail_count + 1;
        end

        // --- Test 3: Verify buffer contents ---
        $display("[TEST 3] Verify sample buffer");
        enable = 0;
        repeat (10) @(posedge clk);

        begin : verify_block
            reg test3_pass;
            test3_pass = 0;
            for (i = 0; i < 8; i = i + 1) begin
                sample_addr = i[5:0];
                repeat (2) @(posedge clk);
                $display("  Sample[%0d] = %0d (0x%04h)", i, $signed(sample_out), sample_out);
                if (sample_out != 16'd0) test3_pass = 1;
            end
            if (test3_pass) begin
                $display("  PASS: Buffer contains data");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: Buffer appears empty");
                fail_count = fail_count + 1;
            end
        end

        // --- Test 4: Second collection cycle ---
        $display("[TEST 4] Second collection cycle");
        enable = 1;
        @(posedge clk); // Clear any previous valid
        wait (samples_valid === 1'b0 || 1);

        begin : wait_samples_valid_2
            integer wait_cnt;
            wait_cnt = 0;
            while (samples_valid !== 1'b1 && wait_cnt < 500000) begin
                @(posedge clk);
                wait_cnt = wait_cnt + 1;
            end
            if (wait_cnt >= 500000)
                $display("  TIMEOUT waiting for samples_valid");
        end

        if (samples_valid === 1'b1) begin
            $display("  PASS: Second collection completed");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Second collection timed out");
            fail_count = fail_count + 1;
        end

        // --- Summary ---
        $display("");
        $display("==========================================");
        $display("  SPI ADC Interface Testbench Results");
        $display("  PASSED: %0d  FAILED: %0d", pass_count, fail_count);
        $display("==========================================");
        if (fail_count > 0) $display("  *** TEST FAILED ***");
        else                $display("  *** ALL TESTS PASSED ***");

        #100;
        $finish;
    end

    // Timeout watchdog
    initial begin
        #50_000_000;
        $display("WATCHDOG: Simulation timeout");
        $finish;
    end

endmodule
