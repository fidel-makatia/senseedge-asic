# SenseEdge on the Zybo Z7-10: scripted Vivado build.
#   cd fpga/zybo_z7_10 && vivado -mode batch -source scripts/build.tcl
# Produces build/senseedge_zybo.xsa (hardware platform for Vitis, bitstream included) and
# build/senseedge_zybo.bit.  Tested flow: Vivado 2022.1 or later with Zynq-7000 support.
# The Digilent board files (https://github.com/Digilent/vivado-boards) make the PS preset
# automatic; without them the PS is configured by hand below (UART1 on MIO 48/49, 33.333 MHz).

set here [file dirname [file normalize [info script]]]
set root [file normalize "$here/.."]
set rtl  [file normalize "$root/../../verilog/rtl"]
set proj_dir "$root/build/vivado"
set part "xc7z010clg400-1"

create_project senseedge_zybo $proj_dir -part $part -force
set bps [get_board_parts -quiet -filter {NAME =~ "*zybo-z7-10*"}]
set have_board [expr {[llength $bps] > 0}]
if {$have_board} {
    set_property board_part [lindex $bps 0] [current_project]
    puts "INFO: using board part [lindex $bps 0]"
} else {
    puts "WARNING: Digilent Zybo Z7-10 board files not found; configuring the PS manually."
}

add_files -norecurse [list \
    $rtl/spi_adc_if.v $rtl/fft_engine.v $rtl/feature_extract.v $rtl/nn_engine.v \
    $rtl/wb_interface.v $rtl/alarm_logic.v $rtl/senseedge_top.v \
    $root/rtl/mcp3201_emu.v $root/rtl/axil_senseedge.v]
add_files -fileset constrs_1 -norecurse $root/constraints/zybo_z7_10.xdc
update_compile_order -fileset sources_1

# ---- block design: Zynq PS (plays the Caravel management core) + the AXI-Lite wrapper ----
create_bd_design "system"
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0]
if {$have_board} {
    apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
        -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps
} else {
    set_property -dict [list \
        CONFIG.PCW_CRYSTAL_PERIPHERAL_FREQMHZ {33.333333} \
        CONFIG.PCW_UART1_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_UART1_UART1_IO {MIO 48 .. 49} \
        CONFIG.PCW_UART1_BAUD_RATE {115200}] $ps
    apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
        -config {make_external "FIXED_IO, DDR" apply_board_preset "0" Master "Disable" Slave "Disable"} $ps
}
# 20 MHz fabric clock = the chip's clock; one AXI GP master; one fabric interrupt
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {20} \
    CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
    CONFIG.PCW_IRQ_F2P_INTR {1}] $ps

set se [create_bd_cell -type module -reference axil_senseedge axil_senseedge_0]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config [list \
    Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} \
    Master {/processing_system7_0/M_AXI_GP0} Slave {/axil_senseedge_0/s_axi} \
    ddr_seg {Auto} intc_ip {New AXI Interconnect} master_apm {0}] \
    [get_bd_intf_pins axil_senseedge_0/s_axi]

# board pins
connect_bd_net [get_bd_pins axil_senseedge_0/spi_miso] [create_bd_port -dir I spi_miso]
connect_bd_net [get_bd_pins axil_senseedge_0/spi_sclk] [create_bd_port -dir O spi_sclk]
connect_bd_net [get_bd_pins axil_senseedge_0/spi_cs_n] [create_bd_port -dir O spi_cs_n]
connect_bd_net [get_bd_pins axil_senseedge_0/led]      [create_bd_port -dir O -from 3 -to 0 led]
connect_bd_net [get_bd_pins axil_senseedge_0/sw]       [create_bd_port -dir I -from 1 -to 0 sw]
connect_bd_net [get_bd_pins axil_senseedge_0/irq]      [get_bd_pins processing_system7_0/IRQ_F2P]

assign_bd_address
set seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces processing_system7_0/Data] -filter {NAME =~ "*axil_senseedge*"}]
set_property offset 0x43C00000 $seg
set_property range 64K $seg
validate_bd_design
save_bd_design

set wrapper [make_wrapper -files [get_files system.bd] -top]
add_files -norecurse $wrapper
set_property top system_wrapper [current_fileset]
update_compile_order -fileset sources_1

launch_runs synth_1 -jobs 4
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} { error "implementation did not complete" }

file mkdir "$root/build"
write_hw_platform -fixed -include_bit -force "$root/build/senseedge_zybo.xsa"
file copy -force [lindex [glob "$proj_dir/senseedge_zybo.runs/impl_1/*.bit"] 0] "$root/build/senseedge_zybo.bit"
report_utilization -file "$root/build/utilization.txt"
report_timing_summary -file "$root/build/timing.txt"
puts "DONE: $root/build/senseedge_zybo.xsa  $root/build/senseedge_zybo.bit"
