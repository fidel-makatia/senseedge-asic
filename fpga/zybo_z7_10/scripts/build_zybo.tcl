# Wrapper around build.tcl for a machine without the Digilent board files on disk: fetch the
# Zybo Z7-10 board definition from the AMD board store (needs network) and then run build.tcl.
#   cd fpga/zybo_z7_10 && vivado -mode batch -source scripts/build_zybo.tcl
set here [file dirname [file normalize [info script]]]
# Boards installed from the store land under the user's xhub directory, which Vivado does not
# add to its board repository by itself; point board.repoPaths there when it exists.
proc add_store_repo {} {
    set base [expr {[info exists ::env(APPDATA)] ? $::env(APPDATA) : "~"}]
    set p [file normalize "$base/Xilinx/Vivado/[version -short]/xhub/board_store/xilinx_board_store/XilinxBoardStore/Vivado/[version -short]/boards"]
    if {[file isdirectory $p]} {
        set_param board.repoPaths [concat [get_param board.repoPaths] [list $p]]
        puts "INFO: board repository += $p"
    }
}
add_store_repo
set bps [get_board_parts -quiet -filter {NAME =~ "*zybo-z7-10*"}]
if {[llength $bps] == 0} {
    puts "INFO: Zybo Z7-10 board part not installed; trying the board store ..."
    if {[catch {
        set store [xhub::get_xstores xilinx_board_store]
        xhub::refresh_catalog $store
        set items [xhub::get_xitems -of_objects $store -filter {NAME =~ "*zybo-z7-10*"}]
        if {[llength $items] == 0} { error "no zybo-z7-10 item in the board store" }
        set item [lindex [lsort -dictionary $items] end]
        xhub::install $item
        puts "INFO: installed $item"
        add_store_repo
    } msg]} {
        puts "WARNING: board store install failed ($msg); build.tcl will configure the PS by hand."
    }
}
puts "INFO: board parts now: [get_board_parts -quiet -filter {NAME =~ "*zybo-z7-10*"}]"
source [file join $here build.tcl]
