########################################################################################################################
# Pin assignments

set_property PACKAGE_PIN Y8 [get_ports clk_200mhz_p]
set_property IOSTANDARD LVCMOS18 [get_ports {trig_out[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {trig_out[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {trig_out[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {trig_out[0]}]
set_property PACKAGE_PIN AB3 [get_ports {trig_out[3]}]
set_property PACKAGE_PIN AA4 [get_ports {trig_out[2]}]
set_property PACKAGE_PIN AA3 [get_ports {trig_out[1]}]
set_property PACKAGE_PIN AB2 [get_ports {trig_out[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN A16 [get_ports {led[3]}]
set_property PACKAGE_PIN A15 [get_ports {led[2]}]
set_property PACKAGE_PIN B15 [get_ports {led[1]}]
set_property PACKAGE_PIN A14 [get_ports {led[0]}]

set_property DRIVE 8 [get_ports {trig_out[3]}]
set_property DRIVE 8 [get_ports {trig_out[2]}]
set_property DRIVE 8 [get_ports {trig_out[1]}]
set_property DRIVE 8 [get_ports {trig_out[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {trig_out[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {trig_out[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {trig_out[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {trig_out[4]}]
set_property SLEW FAST [get_ports {trig_out[0]}]
set_property SLEW FAST [get_ports {trig_out[1]}]
set_property SLEW FAST [get_ports {trig_out[2]}]
set_property SLEW FAST [get_ports {trig_out[3]}]
set_property PACKAGE_PIN AA18 [get_ports {trig_out[7]}]
set_property DRIVE 8 [get_ports {trig_out[7]}]
set_property DRIVE 8 [get_ports {trig_out[6]}]
set_property DRIVE 8 [get_ports {trig_out[5]}]
set_property DRIVE 8 [get_ports {trig_out[4]}]
set_property SLEW FAST [get_ports {trig_out[7]}]
set_property SLEW FAST [get_ports {trig_out[6]}]
set_property SLEW FAST [get_ports {trig_out[5]}]
set_property SLEW FAST [get_ports {trig_out[4]}]
set_property PACKAGE_PIN AB17 [get_ports {trig_out[6]}]
set_property PACKAGE_PIN AB18 [get_ports {trig_out[5]}]
set_property PACKAGE_PIN T15 [get_ports {trig_out[4]}]

set_property IOSTANDARD LVCMOS33 [get_ports {relay_a[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {relay_a[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {relay_a[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {relay_a[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {relay_b[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {relay_b[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {relay_b[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {relay_b[0]}]
set_property PACKAGE_PIN AB22 [get_ports {relay_a[3]}]
set_property PACKAGE_PIN AB16 [get_ports {relay_a[2]}]
set_property PACKAGE_PIN AB21 [get_ports {relay_a[1]}]
set_property PACKAGE_PIN AB15 [get_ports {relay_a[0]}]
set_property PACKAGE_PIN AA21 [get_ports {relay_b[3]}]
set_property PACKAGE_PIN AA16 [get_ports {relay_b[2]}]
set_property PACKAGE_PIN AA20 [get_ports {relay_b[1]}]
set_property PACKAGE_PIN AA15 [get_ports {relay_b[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {trig_out[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {trig_out[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {trig_out[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {trig_out[8]}]
set_property PACKAGE_PIN W22 [get_ports {trig_out[11]}]
set_property PACKAGE_PIN AB20 [get_ports {trig_out[10]}]
set_property PACKAGE_PIN Y22 [get_ports {trig_out[9]}]
set_property PACKAGE_PIN Y17 [get_ports {trig_out[8]}]

set_property IOSTANDARD LVCMOS33 [get_ports rgmii_rst_n]
set_property DRIVE 8 [get_ports rgmii_rst_n]
set_property PACKAGE_PIN F10 [get_ports rgmii_rst_n]

set_property PACKAGE_PIN G11 [get_ports rgmii_rxc]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_rxc]

set_property PACKAGE_PIN E4 [get_ports cdrtrig_n]
set_property PACKAGE_PIN D6 [get_ports gtx_refclk_156m25_p]
set_property PACKAGE_PIN F6 [get_ports gtx_refclk_200m_p]

set_property IOSTANDARD LVDS [get_ports clk_200mhz_p]

set_property PACKAGE_PIN U8 [get_ports {trig_in_p[11]}]
set_property PACKAGE_PIN T9 [get_ports {trig_in_p[10]}]
set_property PACKAGE_PIN AB13 [get_ports {trig_in_p[9]}]
set_property PACKAGE_PIN T11 [get_ports {trig_in_p[8]}]
set_property PACKAGE_PIN R7 [get_ports {trig_in_p[7]}]
set_property PACKAGE_PIN W6 [get_ports {trig_in_p[6]}]
set_property PACKAGE_PIN AA6 [get_ports {trig_in_p[5]}]
set_property PACKAGE_PIN AA9 [get_ports {trig_in_p[4]}]
set_property PACKAGE_PIN U7 [get_ports {trig_in_p[3]}]
set_property PACKAGE_PIN AA5 [get_ports {trig_in_p[2]}]
set_property PACKAGE_PIN AB8 [get_ports {trig_in_p[1]}]
set_property PACKAGE_PIN V10 [get_ports {trig_in_p[0]}]

set_property PACKAGE_PIN G4 [get_ports sfp_rx_p]
set_property PACKAGE_PIN G21 [get_ports sfp_rx_los]
set_property IOSTANDARD LVCMOS33 [get_ports sfp_rx_los]
set_property IOSTANDARD LVCMOS33 [get_ports {sfp_led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sfp_led[0]}]
set_property DRIVE 16 [get_ports {led[3]}]
set_property DRIVE 16 [get_ports {led[2]}]
set_property DRIVE 16 [get_ports {led[1]}]
set_property DRIVE 16 [get_ports {led[0]}]
set_property DRIVE 16 [get_ports {sfp_led[1]}]
set_property DRIVE 16 [get_ports {sfp_led[0]}]
set_property PACKAGE_PIN A13 [get_ports {sfp_led[0]}]
set_property PACKAGE_PIN B13 [get_ports {sfp_led[1]}]

set_property IOSTANDARD LVCMOS33 [get_ports {fan_tach[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {fan_tach[0]}]
set_property PACKAGE_PIN J22 [get_ports {fan_tach[1]}]
set_property PACKAGE_PIN G22 [get_ports {fan_tach[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {qspi_dq[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {qspi_dq[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {qspi_dq[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {qspi_dq[0]}]
set_property PACKAGE_PIN F21 [get_ports {qspi_dq[3]}]
set_property PACKAGE_PIN G20 [get_ports {qspi_dq[2]}]
set_property PACKAGE_PIN E22 [get_ports {qspi_dq[1]}]
set_property PACKAGE_PIN F20 [get_ports {qspi_dq[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports irq]
set_property PACKAGE_PIN J21 [get_ports irq]
set_property PACKAGE_PIN H22 [get_ports qspi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports qspi_cs_n]
set_property PACKAGE_PIN E21 [get_ports qspi_sck]
set_property IOSTANDARD LVCMOS33 [get_ports qspi_sck]
set_property PACKAGE_PIN G12 [get_ports rgmii_mdc]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_mdc]
set_property PACKAGE_PIN E11 [get_ports rgmii_mdio]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_mdio]
set_property PACKAGE_PIN G10 [get_ports rgmii_rx_dv]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_rx_dv]
set_property PACKAGE_PIN B10 [get_ports rgmii_tx_clk]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_tx_clk]
set_property PACKAGE_PIN A10 [get_ports rgmii_tx_en]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_tx_en]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_rxd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_rxd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_rxd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_rxd[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_txd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_txd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_txd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_txd[0]}]
set_property PACKAGE_PIN B11 [get_ports {rgmii_rxd[3]}]
set_property PACKAGE_PIN A11 [get_ports {rgmii_rxd[2]}]
set_property PACKAGE_PIN D11 [get_ports {rgmii_rxd[1]}]
set_property PACKAGE_PIN H12 [get_ports {rgmii_rxd[0]}]
set_property PACKAGE_PIN A9 [get_ports {rgmii_txd[3]}]
set_property PACKAGE_PIN C9 [get_ports {rgmii_txd[2]}]
set_property PACKAGE_PIN A8 [get_ports {rgmii_txd[1]}]
set_property PACKAGE_PIN B8 [get_ports {rgmii_txd[0]}]

########################################################################################################################
# Clock inputs

create_clock -period 5.000 -name clk_200mhz_p -waveform {0.000 2.500} [get_ports clk_200mhz_p]

create_clock -period 6.400 -name gtx_refclk_156m25_p -waveform {0.000 3.200} [get_ports gtx_refclk_156m25_p]
create_clock -period 5.000 -name gtx_refclk_200m_p [get_ports gtx_refclk_200m_p]
create_clock -period 8.000 -name rgmii_rxc -waveform {0.000 4.000} [get_ports rgmii_rxc]

########################################################################################################################
# IO timing

# Naive constraints of 1.2 / 2.8 assume the data and clock are nominally aligned to center with 1.2ns setup/hold time
# This does not appear to actually be the case!
# Actual scope measurements show clock ~912ps after data edge, not 2ns as the datasheet suggests
set_input_delay -clock [get_clocks rgmii_rxc] -clock_fall -min -add_delay 2.300 [get_ports {rgmii_rxd[*]}]
set_input_delay -clock [get_clocks rgmii_rxc] -clock_fall -max -add_delay 2.750 [get_ports {rgmii_rxd[*]}]
set_input_delay -clock [get_clocks rgmii_rxc] -min -add_delay 2.300 [get_ports {rgmii_rxd[*]}]
set_input_delay -clock [get_clocks rgmii_rxc] -max -add_delay 2.750 [get_ports {rgmii_rxd[*]}]
set_input_delay -clock [get_clocks rgmii_rxc] -clock_fall -min -add_delay 2.300 [get_ports rgmii_rx_dv]
set_input_delay -clock [get_clocks rgmii_rxc] -clock_fall -max -add_delay 2.750 [get_ports rgmii_rx_dv]
set_input_delay -clock [get_clocks rgmii_rxc] -min -add_delay 2.300 [get_ports rgmii_rx_dv]
set_input_delay -clock [get_clocks rgmii_rxc] -max -add_delay 2.750 [get_ports rgmii_rx_dv]

########################################################################################################################
# CDC

set_clock_groups -asynchronous -group [get_clocks rgmii_rxc] -group [get_clocks clk_125mhz_raw]
set_clock_groups -asynchronous -group [get_clocks network/xg_transceiver/inst/sfp_transceiver_i/gt0_sfp_transceiver_i/gtxe2_i/RXOUTCLK] -group [get_clocks clk_250mhz_raw]
set_clock_groups -asynchronous -group [get_clocks rgmii_rxc] -group [get_clocks clk_250mhz_raw]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz_raw] -group [get_clocks rgmii_rxc]
set_clock_groups -asynchronous -group [get_clocks clk_250mhz_raw] -group [get_clocks rgmii_rxc]

########################################################################################################################
# Floorplanning

create_pblock pblock_crypt25519
add_cells_to_pblock [get_pblocks pblock_crypt25519] [get_cells -quiet [list crypt25519]]
resize_pblock [get_pblocks pblock_crypt25519] -add {CLOCKREGION_X0Y0:CLOCKREGION_X1Y0}
set_property IS_SOFT FALSE [get_pblocks pblock_crypt25519]

create_pblock pblock_port_xg0
add_cells_to_pblock [get_pblocks pblock_port_xg0] [get_cells -quiet [list network/port_xg0]]
resize_pblock [get_pblocks pblock_port_xg0] -add {SLICE_X36Y101:SLICE_X53Y149}
resize_pblock [get_pblocks pblock_port_xg0] -add {DSP48_X2Y42:DSP48_X2Y59}
resize_pblock [get_pblocks pblock_port_xg0] -add {RAMB18_X2Y50:RAMB18_X2Y59}
resize_pblock [get_pblocks pblock_port_xg0] -add {RAMB36_X2Y25:RAMB36_X2Y29}

create_pblock pblock_port_mgmt0
add_cells_to_pblock [get_pblocks pblock_port_mgmt0] [get_cells -quiet [list mgmt/mgmt0_mdio_obuf mgmt/mgmt0_mdio_txvr mgmt/sync_link_up_txclk network/port_mgmt0]]
resize_pblock [get_pblocks pblock_port_mgmt0] -add {SLICE_X0Y150:SLICE_X11Y178}
resize_pblock [get_pblocks pblock_port_mgmt0] -add {DSP48_X0Y60:DSP48_X0Y69}
resize_pblock [get_pblocks pblock_port_mgmt0] -add {RAMB18_X0Y60:RAMB18_X0Y69}
resize_pblock [get_pblocks pblock_port_mgmt0] -add {RAMB36_X0Y30:RAMB36_X0Y34}
set_property IS_SOFT FALSE [get_pblocks pblock_port_mgmt0]

########################################################################################################################
# JTAG



set_property PACKAGE_PIN C4 [get_ports rx1_n]

create_pblock pblock_xg0_pcs
add_cells_to_pblock [get_pblocks pblock_xg0_pcs] [get_cells -quiet [list network/port_xg0/pcs]]
resize_pblock [get_pblocks pblock_xg0_pcs] -add {SLICE_X44Y101:SLICE_X53Y124}
resize_pblock [get_pblocks pblock_xg0_pcs] -add {DSP48_X2Y42:DSP48_X2Y49}
set_property IS_SOFT FALSE [get_pblocks pblock_xg0_pcs]

set_property PACKAGE_PIN B6 [get_ports rx0_n]

create_pblock pblock_qspi
add_cells_to_pblock [get_pblocks pblock_qspi] [get_cells -quiet [list mgmt/bridge mgmt/tach0 mgmt/tach1]]
resize_pblock [get_pblocks pblock_qspi] -add {SLICE_X0Y50:SLICE_X11Y99}
resize_pblock [get_pblocks pblock_qspi] -add {DSP48_X0Y20:DSP48_X0Y39}
resize_pblock [get_pblocks pblock_qspi] -add {RAMB18_X0Y20:RAMB18_X0Y39}
resize_pblock [get_pblocks pblock_qspi] -add {RAMB36_X0Y10:RAMB36_X0Y19}
set_property IS_SOFT FALSE [get_pblocks pblock_qspi]



create_pblock pblock_rgmii_cdc
add_cells_to_pblock [get_pblocks pblock_rgmii_cdc] [get_cells -quiet [list rx_mux/baset_rx_cdc]]
resize_pblock [get_pblocks pblock_rgmii_cdc] -add {SLICE_X12Y150:SLICE_X35Y163}
resize_pblock [get_pblocks pblock_rgmii_cdc] -add {DSP48_X1Y60:DSP48_X1Y63}
resize_pblock [get_pblocks pblock_rgmii_cdc] -add {RAMB18_X1Y60:RAMB18_X1Y63}
resize_pblock [get_pblocks pblock_rgmii_cdc] -add {RAMB36_X1Y30:RAMB36_X1Y31}
set_property IS_SOFT FALSE [get_pblocks pblock_rgmii_cdc]
create_pblock pblock_rx_mux
add_cells_to_pblock [get_pblocks pblock_rx_mux] [get_cells -quiet [list rx_mux/baser_rx_cdc]]
resize_pblock [get_pblocks pblock_rx_mux] -add {SLICE_X12Y100:SLICE_X23Y149}
resize_pblock [get_pblocks pblock_rx_mux] -add {DSP48_X1Y40:DSP48_X1Y59}
resize_pblock [get_pblocks pblock_rx_mux] -add {RAMB18_X1Y40:RAMB18_X1Y59}
resize_pblock [get_pblocks pblock_rx_mux] -add {RAMB36_X1Y20:RAMB36_X1Y29}
set_property IS_SOFT FALSE [get_pblocks pblock_rx_mux]




create_clock -period 3.103 -name bert/lane1_transceiver/lane1_rxclk_raw -waveform {0.000 1.552} [get_pins bert/lane1_transceiver/gtxchan/RXOUTCLK]
create_clock -period 3.103 -name bert/lane1_transceiver/lane1_txclk_raw -waveform {0.000 1.552} [get_pins bert/lane1_transceiver/gtxchan/TXOUTCLK]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz_raw] -group [get_clocks bert/lane1_transceiver/lane1_rxclk_raw]
set_clock_groups -asynchronous -group [get_clocks clk_250mhz_raw] -group [get_clocks bert/lane1_transceiver/lane1_rxclk_raw]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz_raw] -group [get_clocks bert/lane1_transceiver/lane1_txclk_raw]
set_clock_groups -asynchronous -group [get_clocks clk_250mhz_raw] -group [get_clocks bert/lane1_transceiver/lane1_txclk_raw]
set_clock_groups -asynchronous -group [get_clocks bert/lane1_transceiver/lane1_rxclk_raw] -group [get_clocks clk_125mhz_raw]
set_clock_groups -asynchronous -group [get_clocks bert/lane1_transceiver/lane1_txclk_raw] -group [get_clocks clk_125mhz_raw]
set_clock_groups -asynchronous -group [get_clocks bert/lane1_transceiver/lane1_rxclk_raw] -group [get_clocks clk_250mhz_raw]

create_clock -period 3.103 -name bert/lane0_transceiver/lane0_rxclk_raw -waveform {0.000 1.552} [get_pins bert/lane0_transceiver/gtxchan/RXOUTCLK]
create_clock -period 3.103 -name bert/lane0_transceiver/lane0_txclk_raw -waveform {0.000 1.552} [get_pins bert/lane0_transceiver/gtxchan/TXOUTCLK]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz_raw] -group [get_clocks bert/lane0_transceiver/lane0_rxclk_raw]
set_clock_groups -asynchronous -group [get_clocks clk_250mhz_raw] -group [get_clocks bert/lane0_transceiver/lane0_rxclk_raw]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz_raw] -group [get_clocks bert/lane0_transceiver/lane0_txclk_raw]
set_clock_groups -asynchronous -group [get_clocks clk_250mhz_raw] -group [get_clocks bert/lane0_transceiver/lane0_txclk_raw]
set_clock_groups -asynchronous -group [get_clocks bert/lane0_transceiver/lane0_rxclk_raw] -group [get_clocks clk_125mhz_raw]
set_clock_groups -asynchronous -group [get_clocks bert/lane0_transceiver/lane0_txclk_raw] -group [get_clocks clk_125mhz_raw]
set_clock_groups -asynchronous -group [get_clocks bert/lane0_transceiver/lane0_rxclk_raw] -group [get_clocks clk_250mhz_raw]
set_clock_groups -asynchronous -group [get_clocks bert/lane0_transceiver/lane0_txclk_raw] -group [get_clocks clk_250mhz_raw]



create_clock -period 3.103 -name prbs_transceiver/cdrtrig_rx_clk_raw -waveform {0.000 1.552} [get_pins prbs_transceiver/gtxchan/RXOUTCLK]
create_clock -period 3.103 -name prbs_transceiver/prbs_tx_clk_raw -waveform {0.000 1.552} [get_pins prbs_transceiver/gtxchan/TXOUTCLK]
set_clock_groups -asynchronous -group [get_clocks prbs_transceiver/cdrtrig_rx_clk_raw] -group [get_clocks clk_125mhz_raw]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz_raw] -group [get_clocks prbs_transceiver/cdrtrig_rx_clk_raw]

set_clock_groups -asynchronous -group [get_clocks prbs_transceiver/prbs_tx_clk_raw] -group [get_clocks clk_125mhz_raw]

set_property IOSTANDARD LVCMOS33 [get_ports frontpanel_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports frontpanel_miso]
set_property IOSTANDARD LVCMOS33 [get_ports frontpanel_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports frontpanel_sck]
set_property PULLUP true [get_ports frontpanel_miso]
set_property PACKAGE_PIN N22 [get_ports frontpanel_cs_n]
set_property PACKAGE_PIN K21 [get_ports frontpanel_miso]
set_property PACKAGE_PIN L21 [get_ports frontpanel_mosi]
set_property PACKAGE_PIN K22 [get_ports frontpanel_sck]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_125mhz]
