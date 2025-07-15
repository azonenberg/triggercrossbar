########################################################################################################################
# Timestamp in usercode

set_property BITSTREAM.CONFIG.USR_ACCESS TIMESTAMP [current_design]

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

set_property PACKAGE_PIN C4 [get_ports rx1_n]
set_property PACKAGE_PIN B6 [get_ports rx0_n]

set_property IOSTANDARD LVCMOS33 [get_ports frontpanel_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports frontpanel_miso]
set_property IOSTANDARD LVCMOS33 [get_ports frontpanel_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports frontpanel_sck]
set_property PULLTYPE PULLUP [get_ports frontpanel_miso]
set_property PACKAGE_PIN N22 [get_ports frontpanel_cs_n]
set_property PACKAGE_PIN K21 [get_ports frontpanel_miso]
set_property PACKAGE_PIN L21 [get_ports frontpanel_mosi]
set_property PACKAGE_PIN K22 [get_ports frontpanel_sck]

set_property DRIVE 8 [get_ports frontpanel_cs_n]
set_property DRIVE 8 [get_ports frontpanel_mosi]
set_property DRIVE 8 [get_ports frontpanel_sck]
set_property SLEW SLOW [get_ports frontpanel_cs_n]
set_property SLEW SLOW [get_ports frontpanel_mosi]
set_property SLEW SLOW [get_ports frontpanel_sck]

set_property PULLTYPE PULLUP [get_ports {fan_tach[1]}]
set_property PULLTYPE PULLUP [get_ports {fan_tach[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[0]}]
set_property PACKAGE_PIN N17 [get_ports {pmod_dq[7]}]

set_property PACKAGE_PIN P17 [get_ports {pmod_dq[6]}]
set_property PACKAGE_PIN R18 [get_ports {pmod_dq[5]}]
set_property PACKAGE_PIN P19 [get_ports {pmod_dq[4]}]
set_property PACKAGE_PIN M16 [get_ports {pmod_dq[3]}]
set_property PACKAGE_PIN P16 [get_ports {pmod_dq[2]}]
set_property PACKAGE_PIN R17 [get_ports {pmod_dq[1]}]
set_property PACKAGE_PIN R19 [get_ports {pmod_dq[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {flash_dq[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {flash_dq[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {flash_dq[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {flash_dq[0]}]
set_property PACKAGE_PIN F19 [get_ports {flash_dq[3]}]
set_property PACKAGE_PIN G18 [get_ports {flash_dq[2]}]
set_property PACKAGE_PIN H19 [get_ports {flash_dq[1]}]
set_property PACKAGE_PIN H18 [get_ports {flash_dq[0]}]
set_property PACKAGE_PIN L16 [get_ports flash_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports flash_cs_n]

set_property SLEW SLOW [get_ports {flash_dq[3]}]
set_property SLEW SLOW [get_ports {flash_dq[2]}]
set_property SLEW SLOW [get_ports {flash_dq[1]}]
set_property SLEW SLOW [get_ports {flash_dq[0]}]
set_property SLEW SLOW [get_ports flash_cs_n]

set_property DRIVE 8 [get_ports {flash_dq[3]}]
set_property DRIVE 8 [get_ports {flash_dq[2]}]
set_property DRIVE 8 [get_ports {flash_dq[1]}]
set_property DRIVE 8 [get_ports {flash_dq[0]}]
set_property DRIVE 8 [get_ports flash_cs_n]

########################################################################################################################
# Clock inputs

create_clock -period 5.000 -name clk_200mhz_p -waveform {0.000 2.500} [get_ports clk_200mhz_p]
create_clock -period 6.400 -name gtx_refclk_156m25_p -waveform {0.000 3.200} [get_ports gtx_refclk_156m25_p]
create_clock -period 5.000 -name gtx_refclk_200m_p [get_ports gtx_refclk_200m_p]
create_clock -period 8.000 -name rgmii_rxc -waveform {0.000 4.000} [get_ports rgmii_rxc]

########################################################################################################################
# GTX recovered / PLL clocks

create_clock -period 3.103 -name bert/lane1_transceiver/lane1_rxclk_raw -waveform {0.000 1.552} [get_pins bert/lane1_transceiver/gtxchan/RXOUTCLK]
create_clock -period 3.103 -name bert/lane1_transceiver/lane1_txclk_raw -waveform {0.000 1.552} [get_pins bert/lane1_transceiver/gtxchan/TXOUTCLK]
create_clock -period 3.103 -name bert/lane0_transceiver/lane0_rxclk_raw -waveform {0.000 1.552} [get_pins bert/lane0_transceiver/gtxchan/RXOUTCLK]
create_clock -period 3.103 -name bert/lane0_transceiver/lane0_txclk_raw -waveform {0.000 1.552} [get_pins bert/lane0_transceiver/gtxchan/TXOUTCLK]
create_clock -period 3.103 -name prbs_transceiver/cdrtrig_rx_clk_raw -waveform {0.000 1.552} [get_pins prbs_transceiver/gtxchan/RXOUTCLK]
create_clock -period 3.103 -name prbs_transceiver/prbs_tx_clk_raw -waveform {0.000 1.552} [get_pins prbs_transceiver/gtxchan/TXOUTCLK]

create_generated_clock -name clk_250mhz -source [get_pins clk_main/rgmii_mmcm/CLKIN1] -master_clock [get_clocks clk_200mhz_p] [get_pins clk_main/rgmii_mmcm/CLKOUT4]

########################################################################################################################
# IO timing

# Naive constraints of 1.2 / 2.8 assume the data and clock are nominally aligned to center with 1.2ns setup/hold time
# This does not appear to actually be the case!
# Actual scope measurements show clock ~912ps after data edge, not 2ns as the datasheet suggests

########################################################################################################################
# CDC

set tmp_i_i0 [get_cells -hierarchical -filter { NAME =~  "*sync*" && NAME =~  "*reg_a_ff*" }]
set tmp_i_i1 [get_cells -hierarchical -filter { NAME =~  "*sync*" && NAME =~  "*reg_b*" }]
set tmp_i_i2 [get_cells -hierarchical -filter { NAME =~  "*sync*" && NAME =~  "*tx_a_reg*" }]
set tmp_i_i3 [get_cells -hierarchical -filter { NAME =~  "*sync*" && NAME =~  "*dout1_reg*" }]
set tmp_i_i4 [get_cells -hierarchical -filter { NAME =~  "*sync*" && NAME =~  "*a_ff*" }]
set tmp_i_i5 [get_cells -hierarchical -filter { NAME =~  "*sync*" && NAME =~  "*dout0_reg*" }]
set tmp_i_i6 [get_cells -hierarchical -filter { NAME =~  "*fifomem*" && NAME =~  "*portb_dout_raw_reg*" }]
set tmp_i_i8 [get_cells -hierarchical -filter { NAME =~  "*apb_cdc*" && NAME =~  "*downstream*" }]
set tmp_i_i9 [get_cells -hierarchical -filter { NAME =~  "*apb_cdc*" && NAME =~  "*upstream*" }]

# TODO update comment with what sync this is
set_max_delay -datapath_only -from $tmp_i_i0 -to $tmp_i_i1 4.00
set_bus_skew -from $tmp_i_i0 -to $tmp_i_i1 4.00

# TODO update comment with what sync this is
set_max_delay -datapath_only -from $tmp_i_i2 -to $tmp_i_i3 4.00
set_bus_skew -from $tmp_i_i2 -to $tmp_i_i3 4.00

# TODO update comment with what sync this is
set_max_delay -datapath_only -from $tmp_i_i4 -to $tmp_i_i1 4.00
set_bus_skew -from $tmp_i_i4 -to $tmp_i_i1 4.00

# ThreeStageSynchronizer
set_max_delay -datapath_only -from $tmp_i_i5 -to $tmp_i_i3 4.00
set_bus_skew -from $tmp_i_i5 -to $tmp_i_i3 4.00

# APB_CDC
set_max_delay -datapath_only -from [get_clocks clk_250mhz] -to $tmp_i_i8 4.00
set_bus_skew -from [get_clocks clk_250mhz] -to $tmp_i_i8 4.00

set_max_delay -datapath_only -from $tmp_i_i9 -to [get_clocks clk_250mhz] 4.00
set_bus_skew -from $tmp_i_i9 -to [get_clocks clk_250mhz] 4.00

# dual clock BRAMs in CDC FIFOs
set tmp_i_i7 [get_cells -hierarchical -filter { NAME =~  "*fifomem*" }]
set_false_path -from [get_clocks clk_250mhz] -through $tmp_i_i7 -to $tmp_i_i6

########################################################################################################################
# Synchronized trigger is a timing ignore

set muxsel [get_cells -hierarchical -filter { NAME =~  "*matrix*" && NAME =~  "*muxsel_reg*" }]
set latrig [get_cells -hierarchical -filter { NAME =~  "*bert*" && NAME =~  "*sync_*" && NAME =~  "*dout1_reg*" }]
set_false_path -from $muxsel -to $latrig

########################################################################################################################
# Floorplanning

create_pblock pblock_crypt25519
add_cells_to_pblock [get_pblocks pblock_crypt25519] [get_cells -quiet [list crypt25519 mgmt/apb_regslice_crypt]]
resize_pblock [get_pblocks pblock_crypt25519] -add {CLOCKREGION_X0Y0:CLOCKREGION_X1Y0}
set_property IS_SOFT FALSE [get_pblocks pblock_crypt25519]

create_pblock pblock_port_xg0
add_cells_to_pblock [get_pblocks pblock_port_xg0] [get_cells -quiet [list network/xg0_mac network/xg0_rx_cdc]]
resize_pblock [get_pblocks pblock_port_xg0] -add {SLICE_X36Y75:SLICE_X53Y99}
resize_pblock [get_pblocks pblock_port_xg0] -add {DSP48_X2Y30:DSP48_X2Y39}
resize_pblock [get_pblocks pblock_port_xg0] -add {RAMB18_X2Y30:RAMB18_X2Y39}
resize_pblock [get_pblocks pblock_port_xg0] -add {RAMB36_X2Y15:RAMB36_X2Y19}
set_property IS_SOFT FALSE [get_pblocks pblock_port_xg0]

create_pblock pblock_xg0_pcs
add_cells_to_pblock [get_pblocks pblock_xg0_pcs] [get_cells -quiet [list network/xg0_pcs]]
resize_pblock [get_pblocks pblock_xg0_pcs] -add {SLICE_X44Y90:SLICE_X53Y124}
resize_pblock [get_pblocks pblock_xg0_pcs] -add {DSP48_X2Y36:DSP48_X2Y49}
resize_pblock [get_pblocks pblock_xg0_pcs] -add {RAMB18_X2Y36:RAMB18_X3Y39}
resize_pblock [get_pblocks pblock_xg0_pcs] -add {RAMB36_X2Y18:RAMB36_X3Y19}
set_property IS_SOFT FALSE [get_pblocks pblock_xg0_pcs]

create_pblock pblock_port_mgmt0
add_cells_to_pblock [get_pblocks pblock_port_mgmt0] [get_cells -quiet [list network/port_mgmt0 network/mgmt0_rx_cdc]]
resize_pblock [get_pblocks pblock_port_mgmt0] -add {SLICE_X0Y150:SLICE_X11Y178}
resize_pblock [get_pblocks pblock_port_mgmt0] -add {DSP48_X0Y60:DSP48_X0Y69}
resize_pblock [get_pblocks pblock_port_mgmt0] -add {RAMB18_X0Y60:RAMB18_X0Y69}
resize_pblock [get_pblocks pblock_port_mgmt0] -add {RAMB36_X0Y30:RAMB36_X0Y34}
set_property IS_SOFT FALSE [get_pblocks pblock_port_mgmt0]

create_pblock pblock_qspi
#add_cells_to_pblock [get_pblocks pblock_qspi] [get_cells -quiet [list mgmt/bridge mgmt/tach0 mgmt/tach1]]
resize_pblock [get_pblocks pblock_qspi] -add {SLICE_X0Y50:SLICE_X19Y99}
resize_pblock [get_pblocks pblock_qspi] -add {DSP48_X0Y20:DSP48_X0Y39}
resize_pblock [get_pblocks pblock_qspi] -add {RAMB18_X0Y20:RAMB18_X0Y39}
resize_pblock [get_pblocks pblock_qspi] -add {RAMB36_X0Y10:RAMB36_X0Y19}
set_property IS_SOFT FALSE [get_pblocks pblock_qspi]

create_pblock pblock_bert_cdc
resize_pblock [get_pblocks pblock_bert_cdc] -add {SLICE_X36Y125:SLICE_X53Y149}
add_cells_to_pblock [get_pblocks pblock_bert_cdc] [get_cells -quiet [list bert/lane0_apb_cdc bert/lane0_apb_config_cdc bert/lane1_apb_cdc bert/lane1_apb_config_cdc]]
set_property IS_SOFT FALSE [get_pblocks pblock_bert_cdc]

create_pblock pblock_bert_la
resize_pblock [get_pblocks pblock_bert_la] -add {SLICE_X0Y140:SLICE_X53Y199}
resize_pblock [get_pblocks pblock_bert_la] -add {RAMB18_X0Y56:RAMB18_X2Y79}
resize_pblock [get_pblocks pblock_bert_la] -add {RAMB36_X0Y28:RAMB36_X2Y39}
add_cells_to_pblock [get_pblocks pblock_bert_la] [get_cells -quiet [list bert/lane0_la]]
set_property IS_SOFT FALSE [get_pblocks pblock_bert_la]

create_pblock pblock_cdrtrig_linecode
resize_pblock [get_pblocks pblock_cdrtrig_linecode] -add {CLOCKREGION_X1Y3:CLOCKREGION_X1Y3}
set_property IS_SOFT FALSE [get_pblocks pblock_cdrtrig_linecode]

create_pblock pblock_cdtrtrig_gearboxes
resize_pblock [get_pblocks pblock_cdtrtrig_gearboxes] -add {SLICE_X44Y150:SLICE_X53Y188}
resize_pblock [get_pblocks pblock_cdtrtrig_gearboxes] -add {DSP48_X2Y60:DSP48_X2Y73}
resize_pblock [get_pblocks pblock_cdtrtrig_gearboxes] -add {RAMB18_X2Y60:RAMB18_X2Y73}
resize_pblock [get_pblocks pblock_cdtrtrig_gearboxes] -add {RAMB36_X2Y30:RAMB36_X2Y36}
set_property IS_SOFT FALSE [get_pblocks pblock_cdtrtrig_gearboxes]

########################################################################################################################
# Boot / configuration

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]

########################################################################################################################
# JTAG

set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_250mhz]
