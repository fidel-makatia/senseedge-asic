## SenseEdge on the Digilent Zybo Z7-10 (xc7z010clg400-1).  Pins from the Digilent master XDC.
## The 20 MHz design clock is FCLK_CLK0 from the Zynq PS; no PL clock pin is used.

## Pmod JE (standard Pmod, 3.3 V): ADC on JE1..JE3, power from JE6 (3V3) and JE5 (GND)
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { spi_miso }];   # JE1 <- MCP3201 DOUT
set_property -dict { PACKAGE_PIN W16 IOSTANDARD LVCMOS33 } [get_ports { spi_sclk }];   # JE2 -> MCP3201 CLK
set_property -dict { PACKAGE_PIN J15 IOSTANDARD LVCMOS33 } [get_ports { spi_cs_n }];   # JE3 -> MCP3201 CS
set_property PULLUP true [get_ports { spi_miso }]

## LEDs: LD0 alarm, LD1 status (healthy while enabled), LD2 emulator selected, LD3 toggles per decision
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports { led[0] }];
set_property -dict { PACKAGE_PIN M15 IOSTANDARD LVCMOS33 } [get_ports { led[1] }];
set_property -dict { PACKAGE_PIN G14 IOSTANDARD LVCMOS33 } [get_ports { led[2] }];
set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 } [get_ports { led[3] }];

## Switches: SW0 = 1 selects the external ADC on Pmod JE (0 = on-chip emulator self-test); SW1 spare
set_property -dict { PACKAGE_PIN G15 IOSTANDARD LVCMOS33 } [get_ports { sw[0] }];
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports { sw[1] }];
