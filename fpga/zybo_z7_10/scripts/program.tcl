# SenseEdge on the Zybo Z7-10: program the bitstream and run the bare-metal program over JTAG.
#   xsdb scripts/program.tcl <bitstream.bit> <ps7_init.tcl> <application.elf>
# ps7_init.tcl is written by Vivado into the .xsa (and by Vitis into the platform's hw/ folder);
# it sets the PS clocks (FCLK_CLK0 = 20 MHz), the UART1 MIO pins and the DDR before the ELF runs.
# The board's boot-mode jumper must be on JTAG.
if {$argc != 3} {
    puts "usage: xsdb program.tcl <bit> <ps7_init.tcl> <elf>"
    exit 1
}
lassign $argv bit ps7init elf
foreach f [list $bit $ps7init $elf] {
    if {![file exists $f]} { puts "missing: $f"; exit 1 }
}
connect
after 1000
targets -set -filter {name =~ "APU*"}
rst -system
after 2000
puts "programming PL: $bit"
fpga -file $bit
targets -set -filter {name =~ "APU*"}
puts "ps7_init"
source $ps7init
ps7_init
ps7_post_config
targets -set -filter {name =~ "ARM*#0"}
rst -processor
puts "downloading $elf"
dow $elf
con
puts "RUNNING"
exit
