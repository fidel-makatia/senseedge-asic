// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Top-Level Module
// Predictive Maintenance ASIC with Hardware FFT and Neural Network Inference
// Integrates: SPI ADC → FFT → Feature Extraction → NN Inference → Alarm

`default_nettype none

module senseedge_top (
`ifdef USE_POWER_PINS
    inout vccd1,    // User area 1 1.8V supply
    inout vssd1,    // User area 1 digital ground
`endif

    // Wishbone Slave ports
    input  wire        wb_clk_i,
    input  wire        wb_rst_i,
    input  wire        wbs_stb_i,
    input  wire        wbs_cyc_i,
    input  wire        wbs_we_i,
    input  wire [3:0]  wbs_sel_i,
    input  wire [31:0] wbs_dat_i,
    input  wire [31:0] wbs_adr_i,
    output wire        wbs_ack_o,
    output wire [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  wire [127:0] la_data_in,
    output wire [127:0] la_data_out,
    input  wire [127:0] la_oenb,

    // IOs
    input  wire [15:0] io_in,
    output wire [15:0] io_out,
    output wire [15:0] io_oeb,

    // IRQ
    output wire [2:0] irq
);

    // =========================================================================
    // Clock and Reset
    // =========================================================================
    wire clk = wb_clk_i;
    wire rst = wb_rst_i;

    // =========================================================================
    // Internal Interconnect Wires
    // =========================================================================

    // Control signals (from WB interface)
    wire        enable;
    wire [15:0] clk_div;
    wire [7:0]  alarm_threshold;
    wire [3:0]  fault_count_cfg;

    // SPI ADC ↔ FFT
    wire        samples_valid;
    wire [15:0] sample_data;
    wire [5:0]  sample_addr_from_fft;
    wire [5:0]  sample_count;

    // SPI pins (directly on io_in/io_out)
    wire        spi_clk_out;
    wire        spi_cs_n_out;
    wire        spi_miso_in;

    // FFT ↔ Feature Extraction
    wire        fft_done;
    wire        fft_busy;
    wire [15:0] fft_mag_data;
    wire [4:0]  fft_mag_addr;

    // Feature Extraction ↔ NN
    wire        fe_done;
    wire        fe_busy;
    wire [7:0]  feature_data;
    wire [2:0]  feature_addr_from_nn;

    // NN outputs
    wire        nn_done;
    wire        nn_busy;
    wire [1:0]  class_id;
    wire [7:0]  confidence;

    // Alarm
    wire        alarm_active;
    wire        alarm_irq;
    wire [1:0]  last_fault_class;

    // WB ↔ FFT readback
    wire [4:0]  wb_fft_rd_addr;
    wire [15:0] wb_fft_rd_data;

    // WB ↔ Feature readback
    wire [2:0]  wb_feature_rd_addr;
    wire [7:0]  wb_feature_rd_data;

    // WB ↔ NN weight loading
    wire        wt_wr_en;
    wire [7:0]  wt_wr_addr;
    wire [7:0]  wt_wr_data;

    // =========================================================================
    // GPIO Pin Assignments
    // =========================================================================
    // io_in/io_out[0]  : SPI MISO (input from ADC)
    // io_in/io_out[1]  : SPI CLK  (output to ADC)
    // io_in/io_out[2]  : SPI CS_N (output to ADC)
    // io_in/io_out[3]  : Alarm output (active high)
    // io_in/io_out[4]  : Status LED (healthy=on)
    // io_in/io_out[5]  : UART TX to ESP32 (directly from LA for firmware)
    // io_in/io_out[6]  : UART RX from ESP32 (directly to LA for firmware)
    // [7:15]           : Reserved / unused

    assign spi_miso_in = io_in[0];

    assign io_out[0]  = 1'b0;           // MISO is input
    assign io_out[1]  = spi_clk_out;
    assign io_out[2]  = spi_cs_n_out;
    assign io_out[3]  = alarm_active;
    assign io_out[4]  = enable & ~alarm_active;  // Status LED: on when healthy
    assign io_out[5]  = 1'b1;           // UART TX idle high (firmware drives via LA)
    assign io_out[6]  = 1'b0;           // UART RX is input
    assign io_out[15:7] = 9'd0;

    // Output enable (active low): 0=output, 1=input
    assign io_oeb[0]  = 1'b1;   // MISO = input
    assign io_oeb[1]  = 1'b0;   // SPI CLK = output
    assign io_oeb[2]  = 1'b0;   // SPI CS = output
    assign io_oeb[3]  = 1'b0;   // Alarm = output
    assign io_oeb[4]  = 1'b0;   // Status LED = output
    assign io_oeb[5]  = 1'b0;   // UART TX = output
    assign io_oeb[6]  = 1'b1;   // UART RX = input
    assign io_oeb[15:7] = 9'h1FF; // Unused = inputs

    // =========================================================================
    // Logic Analyzer connections (debug visibility)
    // =========================================================================
    assign la_data_out[1:0]   = class_id;
    assign la_data_out[9:2]   = confidence;
    assign la_data_out[10]    = alarm_active;
    assign la_data_out[11]    = fft_busy;
    assign la_data_out[12]    = nn_busy;
    assign la_data_out[13]    = fe_busy;
    assign la_data_out[14]    = fft_done;
    assign la_data_out[15]    = nn_done;
    assign la_data_out[21:16] = sample_count;
    assign la_data_out[22]    = samples_valid;
    assign la_data_out[23]    = enable;
    assign la_data_out[127:24] = 104'd0;

    // =========================================================================
    // Pipeline Control FSM
    // =========================================================================
    // Autonomous pipeline: SPI collects → FFT runs → features extract → NN infers
    reg fft_start_reg;
    reg fe_start_reg;
    reg nn_start_reg;

    always @(posedge clk) begin
        if (rst) begin
            fft_start_reg <= 1'b0;
            fe_start_reg  <= 1'b0;
            nn_start_reg  <= 1'b0;
        end else begin
            // Default: single-cycle pulses
            fft_start_reg <= 1'b0;
            fe_start_reg  <= 1'b0;
            nn_start_reg  <= 1'b0;

            // Stage transitions
            if (samples_valid && enable)
                fft_start_reg <= 1'b1;
            if (fft_done)
                fe_start_reg <= 1'b1;
            if (fe_done)
                nn_start_reg <= 1'b1;
        end
    end

    // =========================================================================
    // FFT magnitude read mux (WB readback vs Feature Extraction)
    // =========================================================================
    // Feature extraction has priority when busy, otherwise WB can read
    wire [4:0] fft_mag_addr_mux = fe_busy ? fft_mag_addr : wb_fft_rd_addr;
    assign wb_fft_rd_data = fft_mag_data;  // Both read same data

    // Feature read mux (NN vs WB)
    wire [2:0] feature_addr_mux = nn_busy ? feature_addr_from_nn : wb_feature_rd_addr;
    assign wb_feature_rd_data = feature_data;

    // =========================================================================
    // Module Instantiations
    // =========================================================================

    // --- SPI ADC Interface ---
    spi_adc_if u_spi_adc (
        .clk          (clk),
        .rst          (rst),
        .enable       (enable),
        .clk_div      (clk_div),
        .spi_clk      (spi_clk_out),
        .spi_cs_n     (spi_cs_n_out),
        .spi_miso     (spi_miso_in),
        .samples_valid(samples_valid),
        .sample_out   (sample_data),
        .sample_addr  (sample_addr_from_fft),
        .sample_count (sample_count)
    );

    // --- 64-Point FFT Engine ---
    fft_engine u_fft (
        .clk        (clk),
        .rst        (rst),
        .start      (fft_start_reg),
        .sample_in  (sample_data),
        .sample_addr(sample_addr_from_fft),
        .done       (fft_done),
        .mag_out    (fft_mag_data),
        .mag_addr   (fft_mag_addr_mux),
        .busy       (fft_busy)
    );

    // --- Feature Extraction ---
    feature_extract u_feature (
        .clk         (clk),
        .rst         (rst),
        .start       (fe_start_reg),
        .mag_addr    (fft_mag_addr),
        .mag_in      (fft_mag_data),
        .done        (fe_done),
        .feature_out (feature_data),
        .feature_addr(feature_addr_mux),
        .busy        (fe_busy)
    );

    // --- Neural Network Inference Engine ---
    nn_engine u_nn (
        .clk         (clk),
        .rst         (rst),
        .start       (nn_start_reg),
        .feature_in  (feature_data),
        .feature_addr(feature_addr_from_nn),
        .done        (nn_done),
        .class_id    (class_id),
        .confidence  (confidence),
        .busy        (nn_busy),
        .wt_wr_en    (wt_wr_en),
        .wt_wr_addr  (wt_wr_addr),
        .wt_wr_data  (wt_wr_data)
    );

    // --- Alarm Logic ---
    alarm_logic u_alarm (
        .clk                (clk),
        .rst                (rst),
        .classification_done(nn_done),
        .class_id           (class_id),
        .confidence         (confidence),
        .alarm_threshold    (alarm_threshold),
        .fault_count_cfg    (fault_count_cfg),
        .alarm_active       (alarm_active),
        .alarm_irq          (alarm_irq),
        .last_fault_class   (last_fault_class)
    );

    // --- Wishbone Interface ---
    wb_interface u_wb (
        .clk              (clk),
        .rst              (rst),
        .wb_cyc_i         (wbs_cyc_i),
        .wb_stb_i         (wbs_stb_i),
        .wb_we_i          (wbs_we_i),
        .wb_sel_i         (wbs_sel_i),
        .wb_adr_i         (wbs_adr_i),
        .wb_dat_i         (wbs_dat_i),
        .wb_ack_o         (wbs_ack_o),
        .wb_dat_o         (wbs_dat_o),
        .enable           (enable),
        .clk_div          (clk_div),
        .alarm_threshold  (alarm_threshold),
        .fault_count_cfg  (fault_count_cfg),
        .class_id         (class_id),
        .confidence       (confidence),
        .fft_busy         (fft_busy),
        .nn_busy          (nn_busy),
        .fe_busy          (fe_busy),
        .alarm_active     (alarm_active),
        .fft_rd_addr      (wb_fft_rd_addr),
        .fft_rd_data      (wb_fft_rd_data),
        .feature_rd_addr  (wb_feature_rd_addr),
        .feature_rd_data  (wb_feature_rd_data),
        .wt_wr_en         (wt_wr_en),
        .wt_wr_addr       (wt_wr_addr),
        .wt_wr_data       (wt_wr_data),
        .classification_done(nn_done),
        .alarm_irq_in     (alarm_irq),
        .irq              (irq)
    );

endmodule

`default_nettype wire
