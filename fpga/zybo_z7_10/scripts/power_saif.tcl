# Activity-based power estimate of the implemented Zybo design: simulate sim/tb_power.v in
# xsim with recorded vibration windows (build/power/codes_*.hex, written by host/power_stimulus.py),
# capture the switching activity of the wrapper as SAIF, annotate the implemented design and
# report power per hierarchy.  One run per divider in DIVS.
#   cd fpga/zybo_z7_10 && vivado -mode batch -source scripts/power_saif.tcl
set here [file dirname [file normalize [info script]]]
set root [file normalize "$here/.."]
set pdir "$root/build/power"
set DIVS {249 4}
set NWIN 16

open_project "$root/build/vivado/senseedge_zybo.xpr"
set p [file normalize "$::env(APPDATA)/Xilinx/Vivado/[version -short]/xhub/board_store/xilinx_board_store/XilinxBoardStore/Vivado/[version -short]/boards"]
if {[file isdirectory $p]} { set_param board.repoPaths [concat [get_param board.repoPaths] [list $p]] }

# simulation fileset with the power testbench
if {[llength [get_filesets -quiet sim_power]] == 0} { create_fileset -simset sim_power }
set_property SOURCE_SET sources_1 [get_filesets sim_power]
add_files -fileset sim_power -norecurse "$root/sim/tb_power.v"
add_files -fileset sim_power -norecurse "$pdir/weights.hex"
set_property top tb_power [get_filesets sim_power]
set_property top_lib xil_defaultlib [get_filesets sim_power]
current_fileset -simset [get_filesets sim_power]

foreach div $DIVS {
    set codes "$pdir/codes_d${div}.hex"
    if {![file exists $codes]} { error "missing $codes" }
    add_files -fileset sim_power -norecurse $codes
    set saif "$pdir/activity_d${div}.saif"
    set_property -name {xsim.simulate.runtime} -value {-all} -objects [get_filesets sim_power]
    set_property -name {xsim.simulate.saif} -value $saif -objects [get_filesets sim_power]
    set_property -name {xsim.simulate.saif_scope} -value {/tb_power/system_i} -objects [get_filesets sim_power]
    set_property -name {xsim.simulate.saif_all_signals} -value {true} -objects [get_filesets sim_power]
    set_property -name {xsim.simulate.xsim.more_options} -value "-testplusarg div=$div -testplusarg nwin=$NWIN -testplusarg codes=[file tail $codes]" -objects [get_filesets sim_power]
    launch_simulation -simset sim_power -mode behavioral
    close_sim
    if {![file exists $saif]} { error "no SAIF written for D = $div" }
    puts "SAIF_DONE D=$div -> $saif"
}

# annotate the implemented design and report
open_run impl_1
foreach div $DIVS {
    set saif "$pdir/activity_d${div}.saif"
    reset_switching_activity -all
    read_saif $saif -strip_path tb_power -out_file "$pdir/saif_match_d${div}.txt"
    report_power -hierarchical_depth 4 -file "$pdir/power_saif_d${div}.txt"
    puts "POWER_DONE D=$div"
}
puts "ALL_POWER_DONE"
