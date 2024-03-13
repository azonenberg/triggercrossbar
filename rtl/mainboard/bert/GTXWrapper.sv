`timescale 1ns / 1ps
`default_nettype none
module GTXWrapper #
(
	parameter SYSCLK_PERIOD                    = 8	//sysclk_in period, in ns
)
(
	input wire           sysclk_in,
	input wire           soft_reset_tx_in,
	input wire           soft_reset_rx_in,
	input wire           dont_reset_on_data_error_in,

	output wire          tx_fsm_reset_done_out,
	output wire          rx_fsm_reset_done_out,
	input wire           data_valid_in,

	input wire   [8:0]   drpaddr_in,
	input wire           drpclk_in,
	input wire   [15:0]  drpdi_in,
	output wire  [15:0]  drpdo_out,
	input wire           drpen_in,
	output wire          drprdy_out,
	input wire           drpwe_in,

	input wire   [2:0]   loopback_in,
	input wire           eyescanreset_in,
	output wire          eyescandataerror_out,
	input wire           eyescantrigger_in,
	input wire           rxcdrhold_in,
	input wire           rxusrclk_in,
	input wire           rxusrclk2_in,
	output wire  [31:0]  rxdata_out,
	output wire          rxprbserr_out,
	input wire   [2:0]   rxprbssel_in,
	input wire           rxprbscntreset_in,
	input wire           gtxrxp_in,
	input wire           gtxrxn_in,
	input wire           rxbufreset_in,
	output wire  [6:0]   rxmonitorout_out,
	input wire   [1:0]   rxmonitorsel_in,
	output wire          rxoutclk_out,
	output wire          rxoutclkfabric_out,
	input wire           rxpmareset_in,
	input wire           rxpolarity_in,
	output wire          rxresetdone_out,
	input wire   [4:0]   txpostcursor_in,
	input wire   [4:0]   txprecursor_in,
	input wire           txusrclk_in,
	input wire           txusrclk2_in,
	input wire   [2:0]   txrate_in,
	output wire  [1:0]   txbufstatus_out,
	input wire   [3:0]   txdiffctrl_in,
	input wire           txinhibit_in,
	input wire   [31:0]  txdata_in,
	output wire          gtxtxn_out,
	output wire          gtxtxp_out,
	output wire          txoutclk_out,
	output wire          txoutclkfabric_out,
	output wire          txoutclkpcs_out,
	output wire          txratedone_out,
	output wire          txresetdone_out,
	input wire           txpolarity_in,
	input wire   [2:0]   txprbssel_in,

	input wire      qplllock_in,
	input wire      qpllrefclklost_in,
	input wire      qplloutclk_in,
	input wire      qplloutrefclk_in

);

	//Typical CDRLOCK Time is 50,000UI, as per DS183
	localparam RX_CDRLOCK_TIME      = 100000/10.3125;

	localparam integer   WAIT_TIME_CDRLOCK    = RX_CDRLOCK_TIME / SYSCLK_PERIOD;

	wire           gttxreset_t;
	wire           gtrxreset_t;
	wire           txuserrdy_t;
	wire           rxuserrdy_t;

	wire           rxlpmlfhold_i;
	wire           rxlpmhfhold_i;

	reg            rx_cdrlocked;
	integer  rx_cdrlock_counter= 0;

	/*
	gtx_frontlane1_i
	(
		.cpllrefclksel_in(3'b001),
		//-------------------------- Channel - DRP Ports  --------------------------
		.drpaddr_in                     (drpaddr_in),
		.drpclk_in                      (drpclk_in),
		.drpdi_in                       (drpdi_in),
		.drpdo_out                      (drpdo_out),
		.drpen_in                       (drpen_in),
		.drprdy_out                     (drprdy_out),
		.drpwe_in                       (drpwe_in),
		//----------------------------- Clocking Ports -----------------------------
		.qpllclk_in                     (qplloutclk_in),
		.qpllrefclk_in                  (qplloutrefclk_in),
		//------------------------- Digital Monitor Ports --------------------------
		.dmonitorout_out                (dmonitorout_out),
		//----------------------------- Loopback Ports -----------------------------
		.loopback_in                    (loopback_in),
		//------------------- RX Initialization and Reset Ports --------------------
		.eyescanreset_in                (eyescanreset_in),
		.rxuserrdy_in                   (rxuserrdy_t),
		//------------------------ RX Margin Analysis Ports ------------------------
		.eyescandataerror_out           (eyescandataerror_out),
		.eyescantrigger_in              (eyescantrigger_in),
		//----------------------- Receive Ports - CDR Ports ------------------------
		.rxcdrhold_in                   (rxcdrhold_in),
		//---------------- Receive Ports - FPGA RX Interface Ports -----------------
		.rxusrclk_in                    (rxusrclk_in),
		.rxusrclk2_in                   (rxusrclk2_in),
		//---------------- Receive Ports - FPGA RX interface Ports -----------------
		.rxdata_out                     (rxdata_out),
		//----------------- Receive Ports - Pattern Checker Ports ------------------
		.rxprbserr_out                  (rxprbserr_out),
		.rxprbssel_in                   (rxprbssel_in),
		//----------------- Receive Ports - Pattern Checker ports ------------------
		.rxprbscntreset_in              (rxprbscntreset_in),
		//------------------------- Receive Ports - RX AFE -------------------------
		.gtxrxp_in                      (gtxrxp_in),
		//---------------------- Receive Ports - RX AFE Ports ----------------------
		.gtxrxn_in                      (gtxrxn_in),
		//----------------- Receive Ports - RX Buffer Bypass Ports -----------------
		.rxbufreset_in                  (rxbufreset_in),
		//------------------ Receive Ports - RX Equailizer Ports -------------------
		.rxlpmhfhold_in                 (rxlpmhfhold_i),
		.rxlpmlfhold_in                 (rxlpmlfhold_i),
		//------------------- Receive Ports - RX Equalizer Ports -------------------
		.rxdfelpmreset_in               (1'b0),
		.rxmonitorout_out               (rxmonitorout_out),
		.rxmonitorsel_in                (rxmonitorsel_in),
		//------------- Receive Ports - RX Fabric Output Control Ports -------------
		.rxoutclk_out                   (rxoutclk_out),
		.rxoutclkfabric_out             (rxoutclkfabric_out),
		//----------- Receive Ports - RX Initialization and Reset Ports ------------
		.gtrxreset_in                   (gtrxreset_t),
		.rxpmareset_in                  (rxpmareset_in),
		//--------------- Receive Ports - RX Polarity Control Ports ----------------
		.rxpolarity_in                  (rxpolarity_in),
		//------------ Receive Ports -RX Initialization and Reset Ports ------------
		.rxresetdone_out                (rxresetdone_out),
		//---------------------- TX Configurable Driver Ports ----------------------
		.txpostcursor_in                (txpostcursor_in),
		.txprecursor_in                 (txprecursor_in),
		//------------------- TX Initialization and Reset Ports --------------------
		.gttxreset_in                   (gttxreset_t),
		.txuserrdy_in                   (txuserrdy_t),
		//---------------- Transmit Ports - FPGA TX Interface Ports ----------------
		.txusrclk_in                    (txusrclk_in),
		.txusrclk2_in                   (txusrclk2_in),
		//------------------- Transmit Ports - PCI Express Ports -------------------
		.txrate_in                      (txrate_in),
		//-------------------- Transmit Ports - TX Buffer Ports --------------------
		.txbufstatus_out                (txbufstatus_out),
		//------------- Transmit Ports - TX Configurable Driver Ports --------------
		.txdiffctrl_in                  (txdiffctrl_in),
		.txinhibit_in                   (txinhibit_in),
		//---------------- Transmit Ports - TX Data Path interface -----------------
		.txdata_in                      (txdata_in),
		//-------------- Transmit Ports - TX Driver and OOB signaling --------------
		.gtxtxn_out                     (gtxtxn_out),
		.gtxtxp_out                     (gtxtxp_out),
		//--------- Transmit Ports - TX Fabric Clock Output Control Ports ----------
		.txoutclk_out                   (txoutclk_out),
		.txoutclkfabric_out             (txoutclkfabric_out),
		.txoutclkpcs_out                (txoutclkpcs_out),
		.txratedone_out                 (txratedone_out),
		//----------- Transmit Ports - TX Initialization and Reset Ports -----------
		.txresetdone_out                (txresetdone_out),
		//--------------- Transmit Ports - TX Polarity Control Ports ---------------
		.txpolarity_in                  (txpolarity_in),
		//---------------- Transmit Ports - pattern Generator Ports ----------------
		.txprbssel_in                   (txprbssel_in)

	);*/

	//RX Datapath signals
	wire    [63:0]  rxdata_raw;
	assign rxdata_out = rxdata_raw[31:0];

	GTXE2_CHANNEL #
	(
		//Simulation configuration
		.SIM_RECEIVER_DETECT_PASS("TRUE"),
		.SIM_TX_EIDLE_DRIVE_LEVEL("X"),
		.SIM_RESET_SPEEDUP("TRUE"),
		.SIM_CPLLREFCLK_SEL(3'b001),
		.SIM_VERSION("4.0"),

		//Comma alignment configuration(not used)
		.ALIGN_COMMA_DOUBLE("FALSE"),
		.ALIGN_COMMA_ENABLE(10'b1111111111),
		.ALIGN_COMMA_WORD(1),
		.ALIGN_MCOMMA_DET("FALSE"),
		.ALIGN_MCOMMA_VALUE(10'b1010000011),
		.ALIGN_PCOMMA_DET("FALSE"),
		.ALIGN_PCOMMA_VALUE(10'b0101111100),
		.SHOW_REALIGN_COMMA("TRUE"),
		.RXSLIDE_AUTO_WAIT(7),
		.RXSLIDE_MODE("OFF"),
		.RX_SIG_VALID_DLY(10),

		//8B/10B configuration(not used)
		.RX_DISPERR_SEQ_MATCH("FALSE"),
		.DEC_MCOMMA_DETECT("FALSE"),
		.DEC_PCOMMA_DETECT("FALSE"),
		.DEC_VALID_COMMA_ONLY("FALSE"),

		//RX clock correction(not used)
		.CBCC_DATA_SOURCE_SEL("ENCODED"),
		.CLK_COR_SEQ_2_USE("FALSE"),
		.CLK_COR_KEEP_IDLE("FALSE"),
		.CLK_COR_MAX_LAT(19),
		.CLK_COR_MIN_LAT(15),
		.CLK_COR_PRECEDENCE("TRUE"),
		.CLK_COR_REPEAT_WAIT(0),
		.CLK_COR_SEQ_LEN(1),
		.CLK_COR_SEQ_1_ENABLE(4'b1111),
		.CLK_COR_SEQ_1_1(10'b0100000000),
		.CLK_COR_SEQ_1_2(10'b0000000000),
		.CLK_COR_SEQ_1_3(10'b0000000000),
		.CLK_COR_SEQ_1_4(10'b0000000000),
		.CLK_CORRECT_USE("FALSE"),
		.CLK_COR_SEQ_2_ENABLE(4'b1111),
		.CLK_COR_SEQ_2_1(10'b0100000000),
		.CLK_COR_SEQ_2_2(10'b0000000000),
		.CLK_COR_SEQ_2_3(10'b0000000000),
		.CLK_COR_SEQ_2_4(10'b0000000000),

		//RX channel bonding(not used)
		.CHAN_BOND_KEEP_ALIGN("FALSE"),
		.CHAN_BOND_MAX_SKEW(1),
		.CHAN_BOND_SEQ_LEN(1),
		.CHAN_BOND_SEQ_1_1(10'b0000000000),
		.CHAN_BOND_SEQ_1_2(10'b0000000000),
		.CHAN_BOND_SEQ_1_3(10'b0000000000),
		.CHAN_BOND_SEQ_1_4(10'b0000000000),
		.CHAN_BOND_SEQ_1_ENABLE(4'b1111),
		.CHAN_BOND_SEQ_2_1(10'b0000000000),
		.CHAN_BOND_SEQ_2_2(10'b0000000000),
		.CHAN_BOND_SEQ_2_3(10'b0000000000),
		.CHAN_BOND_SEQ_2_4(10'b0000000000),
		.CHAN_BOND_SEQ_2_ENABLE(4'b1111),
		.CHAN_BOND_SEQ_2_USE("FALSE"),
		.FTS_DESKEW_SEQ_ENABLE(4'b1111),
		.FTS_LANE_DESKEW_CFG(4'b1111),
		.FTS_LANE_DESKEW_EN("FALSE"),

		//RX eye scan(TODO tweak)
		.ES_CONTROL(6'b000000),
		.ES_ERRDET_EN("FALSE"),
		.ES_EYE_SCAN_EN("TRUE"),
		.ES_HORZ_OFFSET(12'h000),
		.ES_PMA_CFG(10'b0000000000),
		.ES_PRESCALE(5'b00000),
		.ES_QUALIFIER(80'h00000000000000000000),
		.ES_QUAL_MASK(80'h00000000000000000000),
		.ES_SDATA_MASK(80'h00000000000000000000),
		.ES_VERT_OFFSET(9'b000000000),

		//Fabric bus configuration
		.RX_DATA_WIDTH(32),
		.TX_DATA_WIDTH(32),

		//Internal datapath configuration
		.RX_INT_DATAWIDTH(1),
		.TX_INT_DATAWIDTH(1),

		//RX PMA configuration
		.OUTREFCLK_SEL_INV(2'b11),
		.PMA_RSV(32'h001E7080),	//TODO
		.PMA_RSV2(16'h2070),		//Need to power up eye scan block
		.PMA_RSV3(2'b00),
		.PMA_RSV4(32'h00000000),
		.RX_BIAS_CFG(12'b000000000100),
		.DMONITOR_CFG(24'h000A00),
		.RX_CM_SEL(2'b11),
		.RX_CM_TRIM(3'b010),
		.RX_DEBUG_CFG(12'b000000000000),
		.RX_OS_CFG(13'b0000010000000),
		.TERM_RCAL_CFG(5'b10000),
		.TERM_RCAL_OVRD(1'b0),
		.TST_RSV(32'h00000000),
		.RX_CLK25_DIV(7),
		.TX_CLK25_DIV(7),
		.UCODEER_CLR(1'b0),

		//PCIe configuration
		.PCS_PCIE_EN("FALSE"),

		//RX PCS reserved, no idea what this does
		.PCS_RSVD_ATTR(48'h000000000000),

		//RX buffer configuration(leave at default for now)
		.RXBUF_ADDR_MODE("FAST"),
		.RXBUF_EIDLE_HI_CNT(4'b1000),
		.RXBUF_EIDLE_LO_CNT(4'b0000),
		.RXBUF_EN("TRUE"),
		.RX_BUFFER_CFG(6'b000000),
		.RXBUF_RESET_ON_CB_CHANGE("TRUE"),
		.RXBUF_RESET_ON_COMMAALIGN("FALSE"),
		.RXBUF_RESET_ON_EIDLE("FALSE"),
		.RXBUF_RESET_ON_RATE_CHANGE("TRUE"),
		.RXBUFRESET_TIME(5'b00001),
		.RXBUF_THRESH_OVFLW(61),
		.RXBUF_THRESH_OVRD("FALSE"),
		.RXBUF_THRESH_UNDFLW(4),
		.RXDLY_CFG(16'h001F),
		.RXDLY_LCFG(9'h030),
		.RXDLY_TAP_CFG(16'h0000),
		.RXPH_CFG(24'h000000),
		.RXPHDLY_CFG(24'h084020),
		.RXPH_MONITOR_SEL(5'b00000),
		.RX_XCLK_SEL("RXREC"),
		.RX_DDI_SEL(6'b000000),
		.RX_DEFER_RESET_BUF_EN("TRUE"),

		/*
			RX CDR setup: magic value from transceivers wizard

			Known values(no idea what these mean):
			SATA1		= 72'h03_8000_8BFF_4010_0008
			SATA2		= 72'h03_8800_8BFF_4020_0008
			SATA3		= 72'h03_8000_8BFF_1020_0010
			DP HBR		= 72'h03_8000_8BFF_4020_0008
			DP RBR		= 72'h03_8000_8BFF_4020_0008
			DP HBR2 	= 72'h03_8c00_8bff_2020_0010
			10Gbase-R	= 72'h0b_0000_23ff_1040_0020
		 */
		.RXCDR_CFG(72'h0b000023ff10400020),
		.RXCDR_FR_RESET_ON_EIDLE(1'b0),
		.RXCDR_HOLD_DURING_EIDLE(1'b0),
		.RXCDR_PH_RESET_ON_EIDLE(1'b0),
		.RXCDR_LOCK_CFG(6'b010101),

		//RX reset configuration
		.RXCDRFREQRESET_TIME(5'b00001),
		.RXCDRPHRESET_TIME(5'b00001),
		.RXISCANRESET_TIME(5'b00001),
		.RXPCSRESET_TIME(5'b00001),
		.RXPMARESET_TIME(5'b00011),

		//SATA/SAS OOB(not used)
		.RXOOB_CFG(7'b0000110),
		.SAS_MAX_COM(64),
		.SAS_MIN_COM(36),
		.SATA_BURST_SEQ_LEN(4'b0101),
		.SATA_BURST_VAL(3'b100),
		.SATA_EIDLE_VAL(3'b100),
		.SATA_MAX_BURST(8),
		.SATA_MAX_INIT(21),
		.SATA_MAX_WAKE(7),
		.SATA_MIN_BURST(4),
		.SATA_MIN_INIT(12),
		.SATA_MIN_WAKE(4),

		//Gearbox for 64/66 etc, not used here
		.TXGEARBOX_EN("FALSE"),
		.RXGEARBOX_EN("FALSE"),
		.GEARBOX_MODE(3'b000),

		//TODO configure PRBS stuff
		.RXPRBS_ERR_LOOPBACK(1'b0),

		//Power down config(not used)
		.PD_TRANS_TIME_FROM_P2(12'h03c),
		.PD_TRANS_TIME_NONE_P2(8'h19),
		.PD_TRANS_TIME_TO_P2(8'h64),
		.RX_CLKMUX_PD(1'b1),
		.TX_CLKMUX_PD(1'b1),

		//RX clock out(not sure what this does)
		.TRANS_TIME_RATE(8'h0E),

		//TX buffer
		.TXBUF_EN("TRUE"),
		.TXBUF_RESET_ON_RATE_CHANGE("TRUE"),
		.TXDLY_CFG(16'h001F),
		.TXDLY_LCFG(9'h030),
		.TXDLY_TAP_CFG(16'h0000),
		.TXPH_CFG(16'h0780),
		.TXPHDLY_CFG(24'h084020),
		.TXPH_MONITOR_SEL(5'b00000),
		.TX_XCLK_SEL("TXOUT"),

		//TX driver(may override some of this via DRP)
		.TX_PREDRIVER_MODE(1'b0),
		.TX_DEEMPH0(5'b00000),
		.TX_DEEMPH1(5'b00000),
		.TX_EIDLE_ASSERT_DELAY(3'b110),
		.TX_EIDLE_DEASSERT_DELAY(3'b100),
		.TX_LOOPBACK_DRIVE_HIZ("FALSE"),
		.TX_MAINCURSOR_SEL(1'b0),
		.TX_DRIVE_MODE("DIRECT"),
		.TX_MARGIN_FULL_0(7'b1001110),
		.TX_MARGIN_FULL_1(7'b1001001),
		.TX_MARGIN_FULL_2(7'b1000101),
		.TX_MARGIN_FULL_3(7'b1000010),
		.TX_MARGIN_FULL_4(7'b1000000),
		.TX_MARGIN_LOW_0(7'b1000110),
		.TX_MARGIN_LOW_1(7'b1000100),
		.TX_MARGIN_LOW_2(7'b1000010),
		.TX_MARGIN_LOW_3(7'b1000000),
		.TX_MARGIN_LOW_4(7'b1000000),

		//Magic reset configuration
		.TXPCSRESET_TIME(5'b00001),
		.TXPMARESET_TIME(5'b00001),
		.RXDFELPMRESET_TIME(7'b0001111),

		//Not sure what this is
		.TX_RXDETECT_CFG(14'h1832),
		.TX_RXDETECT_REF(3'b100),

		//CPLL (not yet used, but probably will soon)
		.CPLL_CFG(24'hBC07DC),
		.CPLL_FBDIV(4),
		.CPLL_FBDIV_45(4),
		.CPLL_INIT_CFG(24'h00001E),
		.CPLL_LOCK_CFG(16'h01E8),
		.CPLL_REFCLK_DIV(1),
		.RXOUT_DIV(1),
		.TXOUT_DIV(1),
		.SATA_CPLL_CFG("VCO_3000MHZ"),

		//RX equalizer stuff
		.RXLPM_HF_CFG(14'b00000011110000),
		.RXLPM_LF_CFG(14'b00000011110000),
		.RX_DFE_GAIN_CFG(23'h020FEA),
		.RX_DFE_H2_CFG(12'b000000000000),
		.RX_DFE_H3_CFG(12'b000001000000),
		.RX_DFE_H4_CFG(11'b00011110000),
		.RX_DFE_H5_CFG(11'b00011100000),
		.RX_DFE_KL_CFG(13'b0000011111110),
		.RX_DFE_LPM_CFG(16'h0104),
		.RX_DFE_LPM_HOLD_DURING_EIDLE(1'b0),
		.RX_DFE_UT_CFG(17'b10001111000000000),
		.RX_DFE_VP_CFG(17'b00011111100000011),
		.RX_DFE_KL_CFG2(32'h301148AC),	//magic value for 10Gbase-R, may need others?
		.RX_DFE_XYD_CFG(13'b0000000000000),

		//QPI(not used)
		.TX_QPI_STATUS_EN(1'b0)
	)
	gtxchan
	(
		//Top level transceiver pins
		.GTXRXP                         (gtxrxp_in),
		.GTXRXN                         (gtxrxn_in),
		.GTXTXN                         (gtxtxn_out),
		.GTXTXP                         (gtxtxp_out),

		.RXPOLARITY                     (rxpolarity_in),
		.TXPOLARITY                     (txpolarity_in),

		//CPLL (not yet used, but will be)
		.CPLLFBCLKLOST                  (),
		.CPLLLOCK                       (),
		.CPLLLOCKDETCLK                 (1'b0),
		.CPLLLOCKEN                     (1'b1),
		.CPLLPD                         (1'b1),
		.CPLLREFCLKLOST                 (),
		.CPLLREFCLKSEL                  (3'b001),
		.CPLLRESET                      (1'b0),
		.GTRSVD                         (16'b0000000000000000),
		.PCSRSVDIN                      (16'b0000000000000000),
		.PCSRSVDIN2                     (5'b00000),
		.PMARSVDIN                      (5'b00000),
		.PMARSVDIN2                     (5'b00000),
		.TSTIN                          (20'b11111111111111111111),
		.TSTOUT                         (),

		//Magic reserved port
		.CLKRSVD                        (4'h0),

		//Refclk cascading
		.GTGREFCLK                      (1'b0),
		.GTNORTHREFCLK0                 (1'b0),
		.GTNORTHREFCLK1                 (1'b0),
		.GTREFCLK0                      (1'b0),
		.GTREFCLK1                      (1'b0),
		.GTSOUTHREFCLK0                 (1'b0),
		.GTSOUTHREFCLK1                 (1'b0),

		//DRP
		.DRPADDR                        (drpaddr_in),
		.DRPCLK                         (drpclk_in),
		.DRPDI                          (drpdi_in),
		.DRPDO                          (drpdo_out),
		.DRPEN                          (drpen_in),
		.DRPRDY                         (drprdy_out),
		.DRPWE                          (drpwe_in),

		//Clocks from QPLL
		.GTREFCLKMONITOR                (),
		.QPLLCLK                        (qplloutclk_in),
		.QPLLREFCLK                     (qplloutrefclk_in),
		.RXSYSCLKSEL                    (2'b11),
		.TXSYSCLKSEL                    (2'b11),

		//Digital monitor (not used)
		.DMONITOROUT                    (),

		//TX 8B/10B coder (not used)
		.TX8B10BEN                      (1'b0),
		.TXCHARDISPMODE                 (8'b0),
		.TXCHARDISPVAL                  (8'b0),
		.TXCHARISK                      (8'b0),
		.TX8B10BBYPASS                  (8'b0),

		//RX 8B/10B coder (not used)
		.RX8B10BEN                      (1'b0),
		.SETERRSTATUS                   (1'b0),
		.RXDISPERR                      (),
		.RXNOTINTABLE                   (),
		.RXBYTEISALIGNED                (),
		.RXBYTEREALIGN                  (),
		.RXCOMMADET                     (),
		.RXCOMMADETEN                   (1'b0),
		.RXMCOMMAALIGNEN                (1'b0),
		.RXPCOMMAALIGNEN                (1'b0),
		.RXCHARISCOMMA                  (),
		.RXCHARISK                      (),

		//RX 64/66b gearbox (not used)
		.RXDATAVALID                    (),
		.RXHEADER                       (),
		.RXHEADERVALID                  (),
		.RXSTARTOFSEQ                   (),
		.RXGEARBOXSLIP                  (1'b0),
		.RXSLIDE                        (1'b0),

		//Loopback configuration (not used)
		.LOOPBACK                       (3'b0),

		//PCIe stuff (not used)
		.PHYSTATUS                      (),
		.RXRATE                         (3'b0),
		.RXVALID                        (),

		//Power-down (not used)
		.RXPD                           (2'b00),
		.TXPD                           (2'b00),

		//Resets
		.EYESCANRESET                   (eyescanreset_in),
		.RXUSERRDY                      (rxuserrdy_t),
		.GTRXRESET                      (gtrxreset_t),
		.RXOOBRESET                     (1'b0),
		.RXPCSRESET                     (1'b0),
		.RXPMARESET                     (rxpmareset_in),
		.RXRESETDONE                    (rxresetdone_out),
		.CFGRESET                       (1'b0),
		.GTTXRESET                      (gttxreset_t),
		.PCSRSVDOUT                     (),
		.TXUSERRDY                      (txuserrdy_t),
		.GTRESETSEL                     (1'b0),
		.RESETOVRD                      (1'b0),
		.TXPCSRESET                     (1'b0),
		.TXPMARESET                     (1'b0),
		.TXRESETDONE                    (txresetdone_out),

		//Eye scan ports (not used, we just use the DRP)
		.EYESCANDATAERROR               (),
		.EYESCANMODE                    (1'b0),
		.EYESCANTRIGGER                 (1'b0),

		//RX CDR
		.RXCDRFREQRESET                 (1'b0),
		.RXCDRHOLD                      (1'b0),
		.RXCDRLOCK                      (),
		.RXCDROVRDEN                    (1'b0),
		.RXCDRRESET                     (1'b0),
		.RXCDRRESETRSV                  (1'b0),

		//RX clock correction
		.RXCLKCORCNT                    (),
		.RXRATEDONE                     (),
		.RXOUTCLK                       (rxoutclk_out),
		.RXOUTCLKFABRIC                 (rxoutclkfabric_out),
		.RXOUTCLKPCS                    (),
		.RXOUTCLKSEL                    (3'b010),

		//RX fabric clocks
		.RXUSRCLK                       (rxusrclk_in),
		.RXUSRCLK2                      (rxusrclk2_in),

		//RX data bus
		.RXDATA                         (rxdata_raw),

		//PRBS checker
		.RXPRBSERR                      (rxprbserr_out),
		.RXPRBSSEL                      (rxprbssel_in),
		.RXPRBSCNTRESET                 (1'b0),

		//RX equalizer configuration
		.RXDFEXYDEN                     (1'b1),
		.RXDFEXYDHOLD                   (1'b0),
		.RXDFEXYDOVRDEN                 (1'b0),

		//RX buffer
		.RXBUFRESET                     (rxbufreset_in),
		.RXBUFSTATUS                    (),
		.RXDDIEN                        (1'b0),
		.RXDLYBYPASS                    (1'b1),
		.RXDLYEN                        (1'b0),
		.RXDLYOVRDEN                    (1'b0),
		.RXDLYSRESET                    (1'b0),
		.RXDLYSRESETDONE                (),
		.RXPHALIGN                      (1'b0),
		.RXPHALIGNDONE                  (),
		.RXPHALIGNEN                    (1'b0),
		.RXPHDLYPD                      (1'b0),
		.RXPHDLYRESET                   (1'b0),
		.RXPHMONITOR                    (),
		.RXPHOVRDEN                     (1'b0),
		.RXPHSLIPMONITOR                (),
		.RXSTATUS                       (),

		//RX channel bonding (not used)
		.RXCHANBONDSEQ                  (),
		.RXCHBONDEN                     (1'b0),
		.RXCHBONDLEVEL                  (3'b0),
		.RXCHBONDMASTER                 (1'b0),
		.RXCHBONDO                      (),
		.RXCHBONDSLAVE                  (1'b0),
		.RXCHANISALIGNED                (),
		.RXCHANREALIGN                  (),
		.RXCHBONDI                      (5'b00000),

		//RX equalizer
		.RXLPMEN                        (1'b1),
		.RXLPMHFHOLD                    (rxlpmhfhold_i),
		.RXLPMHFOVRDEN                  (1'b0),
		.RXLPMLFHOLD                    (rxlpmlfhold_i),
		.RXDFEAGCHOLD                   (1'b0),
		.RXDFEAGCOVRDEN                 (1'b0),
		.RXDFECM1EN                     (1'b0),
		.RXDFELFHOLD                    (1'b0),
		.RXDFELFOVRDEN                  (1'b0),
		.RXDFELPMRESET                  (1'b0),
		.RXDFETAP2HOLD                  (1'b0),
		.RXDFETAP2OVRDEN                (1'b0),
		.RXDFETAP3HOLD                  (1'b0),
		.RXDFETAP3OVRDEN                (1'b0),
		.RXDFETAP4HOLD                  (1'b0),
		.RXDFETAP4OVRDEN                (1'b0),
		.RXDFETAP5HOLD                  (1'b0),
		.RXDFETAP5OVRDEN                (1'b0),
		.RXDFEUTHOLD                    (1'b0),
		.RXDFEUTOVRDEN                  (1'b0),
		.RXDFEVPHOLD                    (1'b0),
		.RXDFEVPOVRDEN                  (1'b0),
		.RXDFEVSEN                      (1'b0),
		.RXLPMLFKLOVRDEN                (1'b0),
		.RXMONITOROUT                   (rxmonitorout_out),
		.RXMONITORSEL                   (rxmonitorsel_in),
		.RXOSHOLD                       (1'b0),
		.RXOSOVRDEN                     (1'b0),

		//SATA/SAS OOB signaling (not used)
		.RXCOMSASDET                    (),
		.RXCOMWAKEDET                   (),
		.RXCOMINITDET                   (),
		.RXELECIDLE                     (),
		.RXELECIDLEMODE                 (2'b11),
		.TXCOMFINISH                    (),
		.TXCOMINIT                      (1'b0),
		.TXCOMSAS                       (1'b0),
		.TXCOMWAKE                      (1'b0),
		.TXPDELECIDLEMODE               (1'b0),

		//QPI (not used)
		.RXQPIEN                        (1'b0),
		.RXQPISENN                      (),
		.RXQPISENP                      (),
		.TXQPIBIASEN                    (1'b0),
		.TXQPISTRONGPDOWN               (1'b0),
		.TXQPIWEAKPUP                   (1'b0),
		.TXQPISENN                      (),
		.TXQPISENP                      (),

		//TX phase aligner
		.TXPHDLYTSTCLK                  (1'b0),

		//TX driver configuration
		.TXPOSTCURSOR                   (txpostcursor_in),
		.TXPOSTCURSORINV                (1'b0),
		.TXPRECURSOR                    (txprecursor_in),
		.TXPRECURSORINV                 (1'b0),
		.TXBUFDIFFCTRL                  (3'b100),
		.TXDEEMPH                       (1'b0),
		.TXDIFFCTRL                     (txdiffctrl_in),
		.TXDIFFPD                       (1'b0),
		.TXINHIBIT                      (txinhibit_in),
		.TXMAINCURSOR                   (7'b0000000),
		.TXPISOPD                       (1'b0),

		//TX clocking
		.TXUSRCLK                       (txusrclk_in),
		.TXUSRCLK2                      (txusrclk2_in),
		.TXRATE                         (txrate_in),
		.TXOUTCLK                       (txoutclk_out),
		.TXOUTCLKFABRIC                 (txoutclkfabric_out),
		.TXOUTCLKPCS                    (txoutclkpcs_out),
		.TXOUTCLKSEL                    (3'b010),
		.TXRATEDONE                     (txratedone_out),

		//PCIe driver config
		.TXELECIDLE                     (1'b0),
		.TXMARGIN                       (3'b0),
		.TXSWING                        (1'b0),
		.TXDETECTRX                     (1'b0),

		//TX PRBS generator
		.TXPRBSFORCEERR                 (1'b0),
		.TXPRBSSEL                      (txprbssel_in),

		//TX buffer
		.TXDLYBYPASS                    (1'b1),
		.TXDLYEN                        (1'b0),
		.TXDLYHOLD                      (1'b0),
		.TXDLYOVRDEN                    (1'b0),
		.TXDLYSRESET                    (1'b0),
		.TXDLYSRESETDONE                (),
		.TXDLYUPDOWN                    (1'b0),
		.TXPHALIGN                      (1'b0),
		.TXPHALIGNDONE                  (),
		.TXPHALIGNEN                    (1'b0),
		.TXPHDLYPD                      (1'b0),
		.TXPHDLYRESET                   (1'b0),
		.TXPHINIT                       (1'b0),
		.TXPHINITDONE                   (),
		.TXPHOVRDEN                     (1'b0),
		.TXBUFSTATUS                    (txbufstatus_out),

		//TX fabric data
		.TXDATA                         ({32'h0, txdata_in}),

		//TX gearbox (not used)
		.TXGEARBOXREADY                 (),
		.TXHEADER                       (3'b0),
		.TXSEQUENCE                     (7'b0),
		.TXSTARTSEQ                     (1'b0)
	);

	gtx_frontlane1_TX_STARTUP_FSM #
	(
		.EXAMPLE_SIMULATION       (0),
		.STABLE_CLOCK_PERIOD      (SYSCLK_PERIOD),         // Period of the stable clock driving this state-machine, unit is [ns]
		.RETRY_COUNTER_BITWIDTH   (8),
		.TX_QPLL_USED             ("TRUE"),                      // the TX and RX Reset FSMs must
		.RX_QPLL_USED             ("TRUE"),                      // share these two generic values
		.PHASE_ALIGNMENT_MANUAL   ("FALSE")              		 // Decision if a manual phase-alignment is necessary or the automatic
																 // is enough. For single-lane applications the automatic alignment is
																 // sufficient
	)
	txresetfsm_i
	(
		.STABLE_CLOCK                   (sysclk_in),
		.TXUSERCLK                      (txusrclk_in),
		.SOFT_RESET                     (soft_reset_tx_in),
		.QPLLREFCLKLOST                 (qpllrefclklost_in),
		.CPLLREFCLKLOST                 (1'b0),
		.QPLLLOCK                       (qplllock_in),
		.CPLLLOCK                       (1'b1),
		.TXRESETDONE                    (txresetdone_out),
		.MMCM_LOCK                      (1'b1),
		.GTTXRESET                      (gttxreset_t),
		.MMCM_RESET                     (),
		.QPLL_RESET                     (),
		.CPLL_RESET                     (),
		.TX_FSM_RESET_DONE              (tx_fsm_reset_done_out),
		.TXUSERRDY                      (txuserrdy_t),
		.RUN_PHALIGNMENT                (),
		.RESET_PHALIGNMENT              (),
		.PHALIGNMENT_DONE               (1'b1),
		.RETRY_COUNTER                  ()
	);

	gtx_frontlane1_RX_STARTUP_FSM  #
	(
		.EXAMPLE_SIMULATION       (0),
		.EQ_MODE                  ("LPM"),                  		//Rx Equalization Mode - Set to DFE or LPM
		.STABLE_CLOCK_PERIOD      (SYSCLK_PERIOD),         		  	//Period of the stable clock driving this state-machine, unit is [ns]
		.RETRY_COUNTER_BITWIDTH   (8),
		.TX_QPLL_USED             ("TRUE"),                         // the TX and RX Reset FSMs must
		.RX_QPLL_USED             ("TRUE"),                         // share these two generic values
		.PHASE_ALIGNMENT_MANUAL   ("FALSE")              			// Decision if a manual phase-alignment is necessary or the automatic
																	// is enough. For single-lane applications the automatic alignment is
																	// sufficient
	)
	rxresetfsm_i
	(
		.STABLE_CLOCK                   (sysclk_in),
		.RXUSERCLK                      (rxusrclk_in),
		.SOFT_RESET                     (soft_reset_rx_in),
		.DONT_RESET_ON_DATA_ERROR       (dont_reset_on_data_error_in),
		.QPLLREFCLKLOST                 (qpllrefclklost_in),
		.CPLLREFCLKLOST                 (1'b0),
		.QPLLLOCK                       (qplllock_in),
		.CPLLLOCK                       (1'b1),
		.RXRESETDONE                    (rxresetdone_out),
		.MMCM_LOCK                      (1'b1),
		.RECCLK_STABLE                  (rx_cdrlocked),
		.RECCLK_MONITOR_RESTART         (1'b0),
		.DATA_VALID                     (data_valid_in),
		.TXUSERRDY                      (1'b1),
		.GTRXRESET                      (gtrxreset_t),
		.MMCM_RESET                     (),
		.QPLL_RESET                     (),
		.CPLL_RESET                     (),
		.RX_FSM_RESET_DONE              (rx_fsm_reset_done_out),
		.RXUSERRDY                      (rxuserrdy_t),
		.RUN_PHALIGNMENT                (),
		.RESET_PHALIGNMENT              (),
		.PHALIGNMENT_DONE               (1'b1),
		.RXDFEAGCHOLD                   (),
		.RXDFELFHOLD                    (),
		.RXLPMLFHOLD                    (rxlpmlfhold_i),
		.RXLPMHFHOLD                    (rxlpmhfhold_i),
		.RETRY_COUNTER                  ()
	);

	always @(posedge sysclk_in) begin
		if(gtrxreset_t) begin
			rx_cdrlocked       <= 1'b0;
			rx_cdrlock_counter <= 0;
		end
		else if (rx_cdrlock_counter == WAIT_TIME_CDRLOCK) begin
			rx_cdrlocked       <= 1'b1;
			rx_cdrlock_counter <= rx_cdrlock_counter;
		end
		else
			rx_cdrlock_counter <= rx_cdrlock_counter + 1;
	end

endmodule
