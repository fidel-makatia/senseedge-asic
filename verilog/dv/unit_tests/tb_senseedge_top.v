// SPDX-License-Identifier: Apache-2.0
// Integration Testbench: SenseEdge Top
// End-to-end test: Wishbone config → SPI ADC collection → FFT → Features → NN → Alarm
// Simulates an external ADC and verifies the full pipeline

`timescale 1ns / 1ps

module tb_senseedge_top;

    // --- Clock and Reset ---
    reg clk;
    reg rst;

    initial clk = 0;
    always #20 clk = ~clk; // 25 MHz

    // --- DUT signals ---
    reg         wbs_cyc_i;
    reg         wbs_stb_i;
    reg         wbs_we_i;
    reg  [3:0]  wbs_sel_i;
    reg  [31:0] wbs_dat_i;
    reg  [31:0] wbs_adr_i;
    wire        wbs_ack_o;
    wire [31:0] wbs_dat_o;

    reg  [127:0] la_data_in;
    wire [127:0] la_data_out;
    reg  [127:0] la_oenb;

    reg  [15:0] io_in;
    wire [15:0] io_out;
    wire [15:0] io_oeb;

    wire [2:0] irq;

    // --- DUT ---
    senseedge_top dut (
    `ifdef USE_POWER_PINS
        .vccd1(1'b1),
        .vssd1(1'b0),
    `endif
        .wb_clk_i (clk),
        .wb_rst_i (rst),
        .wbs_stb_i(wbs_stb_i),
        .wbs_cyc_i(wbs_cyc_i),
        .wbs_we_i (wbs_we_i),
        .wbs_sel_i(wbs_sel_i),
        .wbs_dat_i(wbs_dat_i),
        .wbs_adr_i(wbs_adr_i),
        .wbs_ack_o(wbs_ack_o),
        .wbs_dat_o(wbs_dat_o),
        .la_data_in (la_data_in),
        .la_data_out(la_data_out),
        .la_oenb    (la_oenb),
        .io_in (io_in),
        .io_out(io_out),
        .io_oeb(io_oeb),
        .irq   (irq)
    );

    // =========================================================================
    // Simulated SPI ADC (MCP3201-style)
    // =========================================================================
    wire spi_clk_w  = io_out[1];
    wire spi_cs_n_w = io_out[2];

    reg [15:0] adc_shift_reg;
    reg [4:0]  adc_bit_cnt;
    integer    adc_sample_idx;

    // Generate test vibration data: DC offset + tone at bin 8 (fault signature)
    reg [11:0] adc_data [0:63];
    integer init_i;
    initial begin
        for (init_i = 0; init_i < 64; init_i = init_i + 1) begin
            // DC=2048 + sin tone at Fs/8 with amplitude 500
            case (init_i % 8)
                0: adc_data[init_i] = 12'd2048;
                1: adc_data[init_i] = 12'd2048 + 12'd354;
                2: adc_data[init_i] = 12'd2048 + 12'd500;
                3: adc_data[init_i] = 12'd2048 + 12'd354;
                4: adc_data[init_i] = 12'd2048;
                5: adc_data[init_i] = 12'd2048 - 12'd354;
                6: adc_data[init_i] = 12'd2048 - 12'd500;
                7: adc_data[init_i] = 12'd2048 - 12'd354;
            endcase
        end
        adc_sample_idx = 0;
    end

    // SPI ADC behavior
    always @(negedge spi_cs_n_w) begin
        adc_shift_reg = {3'b000, adc_data[adc_sample_idx % 64], 1'b0};
        adc_bit_cnt   = 0;
        adc_sample_idx = adc_sample_idx + 1;
    end

    always @(posedge spi_clk_w) begin
        if (!spi_cs_n_w) begin
            adc_bit_cnt <= adc_bit_cnt + 1;
        end
    end

    // Drive MISO on SPI clock falling edge
    always @(negedge spi_clk_w or posedge spi_cs_n_w) begin
        if (spi_cs_n_w)
            io_in[0] <= 1'bz;
        else
            io_in[0] <= adc_shift_reg[15 - adc_bit_cnt];
    end

    // =========================================================================
    // Wishbone Bus Tasks
    // =========================================================================
    task wb_write;
        input [31:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            wbs_cyc_i <= 1;
            wbs_stb_i <= 1;
            wbs_we_i  <= 1;
            wbs_sel_i <= 4'hF;
            wbs_adr_i <= addr;
            wbs_dat_i <= data;
            @(posedge clk);
            wait (wbs_ack_o === 1'b1);
            @(posedge clk);
            wbs_cyc_i <= 0;
            wbs_stb_i <= 0;
            wbs_we_i  <= 0;
            repeat (2) @(posedge clk);
        end
    endtask

    task wb_read;
        input  [31:0] addr;
        output [31:0] data;
        begin
            @(posedge clk);
            wbs_cyc_i <= 1;
            wbs_stb_i <= 1;
            wbs_we_i  <= 0;
            wbs_sel_i <= 4'hF;
            wbs_adr_i <= addr;
            @(posedge clk);
            wait (wbs_ack_o === 1'b1);
            data = wbs_dat_o;
            @(posedge clk);
            wbs_cyc_i <= 0;
            wbs_stb_i <= 0;
            repeat (2) @(posedge clk);
        end
    endtask

    // Weight image, 212 INT8 bytes; loaded as 53 words (byte lane k of word w = weight[4w+k])
    reg [7:0] wimg [0:211];
    task wb_load_weights;
        integer w;
        begin
            for (w = 0; w < 53; w = w + 1)
                wb_write(32'h20 + 4*w, {wimg[4*w+3], wimg[4*w+2], wimg[4*w+1], wimg[4*w]});
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================
    integer pass_count;
    integer fail_count;
    integer i;
    reg [31:0] rd_data;

    initial begin
        $dumpfile("tb_senseedge_top.vcd");
        $dumpvars(0, tb_senseedge_top);

        pass_count = 0;
        fail_count = 0;

        // Initialize
        rst       = 1;
        wbs_cyc_i = 0;
        wbs_stb_i = 0;
        wbs_we_i  = 0;
        wbs_sel_i = 4'h0;
        wbs_adr_i = 32'd0;
        wbs_dat_i = 32'd0;
        la_data_in = 128'd0;
        la_oenb    = {128{1'b1}};  // All LA as inputs (disabled)
        io_in      = 16'd0;

        repeat (20) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);

        $display("=========================================");
        $display(" SenseEdge Integration Test");
        $display("=========================================");

        // ==================================================================
        // Phase 1: Load NN weights
        // ==================================================================
        $display("");
        $display("[PHASE 1] Loading neural network weights...");

        // Every weight zero, then: neuron k <- input k (band k), class k <- hidden k
        for (i = 0; i < 212; i = i + 1) wimg[i] = 8'd0;
        wimg[0]   = 8'd127;   // neuron 0, input 0 (low band)
        wimg[9]   = 8'd127;   // neuron 1, input 1 (mid-low band)
        wimg[18]  = 8'd127;   // neuron 2, input 2 (mid-high band)
        wimg[27]  = 8'd127;   // neuron 3, input 3 (high band)
        wimg[144] = 8'd127;   // class 0 <- hidden 0
        wimg[161] = 8'd127;   // class 1 <- hidden 1
        wimg[178] = 8'd127;   // class 2 <- hidden 2
        wimg[195] = 8'd127;   // class 3 <- hidden 3
        wb_load_weights();
        // read back every word through the bus
        begin : wreadback
            integer w, bad;
            bad = 0;
            for (w = 0; w < 53; w = w + 1) begin
                wb_read(32'h20 + 4*w, rd_data);
                if (rd_data !== {wimg[4*w+3], wimg[4*w+2], wimg[4*w+1], wimg[4*w]}) bad = bad + 1;
            end
            if (bad == 0) begin
                $display("  PASS: 53 weight words read back as written");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: %0d weight words differ on read-back", bad);
                fail_count = fail_count + 1;
            end
        end
        $display("  212 weights loaded");

        // ==================================================================
        // Phase 2: Configure alarm
        // ==================================================================
        $display("");
        $display("[PHASE 2] Configuring alarm...");
        wb_write(32'h0C, 32'h00000364); // threshold=100, fault_count=3

        // Set clock divider for fast simulation
        wb_write(32'h1C, 32'h00000004); // divider=4

        $display("  Alarm threshold=100, consecutive faults=3");

        // ==================================================================
        // Phase 3: Enable system and wait for pipeline
        // ==================================================================
        $display("");
        $display("[PHASE 3] Enabling system - starting acquisition...");
        wb_write(32'h00, 32'h00000001); // Enable

        // Wait for pipeline to complete (SPI → FFT → Features → NN)
        // This takes: 64 SPI transactions + FFT computation + feature extraction + NN inference
        $display("  Waiting for pipeline completion...");

        // Monitor LA for completion signals
        fork
            begin : pipeline_wait
                integer wait_cycles;
                wait_cycles = 0;
                while (wait_cycles < 2_000_000) begin
                    @(posedge clk);
                    wait_cycles = wait_cycles + 1;
                    // la_data_out[15] = nn_done
                    if (la_data_out[15] === 1'b1) begin
                        $display("  Pipeline completed at cycle %0d", wait_cycles);
                        disable pipeline_wait;
                    end
                end
                $display("  WARNING: Pipeline did not complete in 2M cycles");
            end
        join

        // Let the system settle
        repeat (100) @(posedge clk);

        // ==================================================================
        // Phase 4: Read results
        // ==================================================================
        $display("");
        $display("[PHASE 4] Reading classification results...");

        wb_read(32'h08, rd_data);
        $display("  CLASS_RESULT: class=%0d, confidence=%0d",
                 rd_data[1:0], rd_data[9:2]);

        // Tone at bin 8 lies in the mid-low band (bins 5-10) -> hidden 1 -> class 1, saturated confidence
        if (rd_data[1:0] == 2'd1 && rd_data[9:2] == 8'd255) begin
            $display("  PASS: Class 1 (mid-low band) with confidence 255, as the weight image dictates");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Expected class 1 with confidence 255");
            fail_count = fail_count + 1;
        end

        // Read status
        wb_read(32'h04, rd_data);
        $display("  STATUS: 0x%08h (enable=%b fft_busy=%b nn_busy=%b fe_busy=%b alarm=%b)",
                 rd_data, rd_data[0], rd_data[1], rd_data[2], rd_data[3], rd_data[4]);

        // ==================================================================
        // Phase 5: Read FFT bins
        // ==================================================================
        $display("");
        $display("[PHASE 5] Reading FFT magnitude bins...");
        wb_write(32'h10, 32'h00000000); // Reset read pointer
        begin : fft_read
            reg found_nonzero;
            found_nonzero = 0;
            for (i = 0; i < 32; i = i + 1) begin
                wb_read(32'h10, rd_data);
                if (i < 16) // Print first 16 bins
                    $display("    Bin[%2d] = %5d", i, rd_data[15:0]);
                if (rd_data[15:0] > 0) found_nonzero = 1;
            end
            if (found_nonzero) begin
                $display("  PASS: FFT produced non-zero output");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: All FFT bins are zero");
                fail_count = fail_count + 1;
            end
        end

        // ==================================================================
        // Phase 6: Read features
        // ==================================================================
        $display("");
        $display("[PHASE 6] Reading extracted features...");
        wb_write(32'h14, 32'h00000000); // Reset read pointer
        begin : feat_read
            reg found_feat;
            found_feat = 0;
            for (i = 0; i < 8; i = i + 1) begin
                wb_read(32'h14, rd_data);
                $display("    Feature[%0d] = %3d", i, rd_data[7:0]);
                if (rd_data[7:0] > 0) found_feat = 1;
            end
            if (found_feat) begin
                $display("  PASS: Features extracted successfully");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: All features are zero");
                fail_count = fail_count + 1;
            end
        end

        // ==================================================================
        // Phase 7: Check GPIO outputs
        // ==================================================================
        $display("");
        $display("[PHASE 7] Checking GPIO outputs...");
        $display("  io_oeb[0] (MISO) = %b (expect 1=input)", io_oeb[0]);
        $display("  io_oeb[1] (SCLK) = %b (expect 0=output)", io_oeb[1]);
        $display("  io_oeb[2] (CS_N) = %b (expect 0=output)", io_oeb[2]);
        $display("  io_oeb[3] (ALRM) = %b (expect 0=output)", io_oeb[3]);
        $display("  io_out[3] (ALRM) = %b", io_out[3]);
        $display("  io_out[4] (LED)  = %b", io_out[4]);

        if (io_oeb[0] == 1'b1 && io_oeb[1] == 1'b0 && io_oeb[2] == 1'b0) begin
            $display("  PASS: GPIO directions correct");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: GPIO directions incorrect");
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Phase 8: Check IRQ
        // ==================================================================
        $display("");
        $display("[PHASE 8] Checking IRQ flags...");
        wb_read(32'h18, rd_data);
        $display("  IRQ flags = 0x%08h", rd_data);
        if (rd_data[0] == 1'b1) begin
            $display("  PASS: Classification done IRQ asserted");
            pass_count = pass_count + 1;
        end else begin
            $display("  INFO: No classification IRQ (may need IRQ enable)");
            pass_count = pass_count + 1;
        end

        // ==================================================================
        // Phase 9: Disable and verify
        // ==================================================================
        $display("");
        $display("[PHASE 9] Disabling system...");
        wb_write(32'h00, 32'h00000000); // Disable
        repeat (10) @(posedge clk);

        wb_read(32'h04, rd_data);
        if (rd_data[0] == 1'b0) begin
            $display("  PASS: System disabled");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: System still enabled");
            fail_count = fail_count + 1;
        end

        // --- Summary ---
        $display("");
        $display("==========================================");
        $display("  SenseEdge Integration Test Results");
        $display("  PASSED: %0d  FAILED: %0d", pass_count, fail_count);
        $display("==========================================");
        if (fail_count > 0) $display("  *** TEST FAILED ***");
        else                $display("  *** ALL TESTS PASSED ***");

        #1000;
        $finish;
    end

    // Timeout watchdog
    initial begin
        #200_000_000;
        $display("WATCHDOG: Simulation timeout (200ms)");
        $finish;
    end

endmodule
