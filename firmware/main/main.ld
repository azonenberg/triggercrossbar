MEMORY
{
	OCTOSPI_APB1(RW):	ORIGIN = 0x90000000, LENGTH = 32K
	OCTOSPI_APB2(RW):	ORIGIN = 0x90008000, LENGTH = 32K
}

SECTIONS
{
	/* Small address branch */
	.sfr_fpga_apb1  (NOLOAD):
	{
		. = ALIGN(1024);
		KEEP (*(.fdevinfo))
		. = ALIGN(1024);
		KEEP (*(.fgpio0))
		. = ALIGN(1024);
		KEEP (*(.fgpio1))
		. = ALIGN(1024);
		KEEP (*(.fmdio))
		. = ALIGN(1024);
		KEEP (*(.frelay))
		. = ALIGN(1024);
		KEEP (*(.ffrontspi))
		. = ALIGN(1024);
		KEEP (*(.fmuxsel))
		. = ALIGN(1024);
		KEEP (*(.fcurve25519))
		. = ALIGN(1024);
		KEEP (*(.firqstat))
		. = ALIGN(1024);
		KEEP (*(.fspi1))

	} > OCTOSPI_APB1

	/* Large address branch */
	.sfr_fpga_apb2  (NOLOAD):
	{
		. = ALIGN(4096);
		KEEP (*(.xgtx))
		. = ALIGN(4096);
		KEEP (*(.gtx))
		. = ALIGN(4096);
		KEEP (*(.ethrx))
		. = ALIGN(4096);

		/* BERT bridge (0x100 per node) */
	} > OCTOSPI_APB2
}
