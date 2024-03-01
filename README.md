# PROJECT STATUS

You probably don't want to build this as is, there's a couple of bugs. May or may not respin later on with fixes, for now I'm just bodging my prototype.

# What is it?

When finished, it will be a 1U rack-mountable appliance with a TBD number of BNC and/or SMA connectors on the back side for trigger inputs and outputs. Some will be input only, some output only, and some bidirectional. Exact mix is TBD.

Additionally, a front panel SMA input will be connected through a precision 10 GHz comparator (ADI HMC675LP3E) to an FPGA transceiver input, allowing single ended CDR triggering.

There may or may not be front panel SMA inputs/outputs connected to otherwise unused transceiver channels for expansion or additional trigger capabilities.

An Ethernet SCPI interface will allow configuration of an FPGA-based crossbar, with the internal CDR trigger as well as all of the input channels available as inputs. Any input can be forwarded to any output, allowing arbitrary cross-triggering of attached instruments without the need to recable.

Front panel LEDs with pulse stretching will display the states of trigger in/out ports.

This system is intended in part as a testbed to de-risk several future projects (validating stackup and power supply design etc), and to use up an FPGA that I had lying around the lab. Only one unit is ever expected to be built, so a functionally equivalent platform could definitely be built for lower cost.

# Physical architecture

# Trigger ports

Koaxis AL62-CC086F-RL10-6.00-MK (6" CC086F hand formable FEP jacket SMA - SMPM), length open to change based on final mechanical and board design.

## Input range

The range of the inputs should be sufficient to accommodate all common T&M instruments, which will likely require variable thresholding.

Representative instrument trigger output specs:

* PicoScope 6000E (bidirectional): not specified, need to measure
* PicoVNA 108: 3.3V into high-Z
* Siglent SSG5000X (bidirectional): 5V TTL for trigger, 3.3V CMOS for pulse
* Siglent SDG7000A: 5V TTL (voh = 3.8V)
* Teledyne LeCroy (all recent models): 1V into 1M ohm / 500 mV into 50 ohm by default, adjustable a bit?

## Output range

The range of the outputs should be compatible with all common T&M instruments, which will likely require variable swing.

Representative instrument trigger input specs:

* PicoScope 6000E (bidirectional): 2.5V CMOS high-z input, fixed 1.25V threshold
* PicoVNA 108: 3.3V CMOS high-Z, nominal 1.5V threshold
* Siglent SSG5000X (bidirectional): 5V TTL for trigger, 3.3V CMOS for pulse
* Siglent SDG7000A: 5V TTL (Vih = 2 to 5.5V)
* Teledyne LeCroy (all recent models): adjustable threshold up to +/- 400 mV or +/- 4V in 1x / 10x mode respectively

# Input buffer design

We need a total of seven inputs today, plan for at least ten to provide some margin
