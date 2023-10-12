# Intermediate Bus Converter

This board takes in 48V DC from an external supply on a 6-pin Molex Mini-Fit Jr connector (p/n 0039291067).

Initial development will be using bench supplies and a Mean Well GST280A48-C6P however the system is intended to support
rack or lab wide DC bus architectures as well.

# Output specs

12V DC at up to 6A (72W)

Expected efficiency at 48 -> 12V is 93%
So for 72W out we will draw 77.4W in (1.61A) and dissipate 5.4W in the converter.

# Monitoring subsystem

TPS7A2533DRVR 3.3V LDO driven from 12V output

300 mA spec, but target << 1W dissipation
With 12V -> 3.3V we drop 8.7V so we'll hit 1W at only 115 mA

## Connector

Management header to main logic board:
* 3.3V standby LDO rail (thermally limited to 115 mA, aim for much less)
* I2C bus with MCU and temp sensor
* 3.3V active high enable for main 12V rail
* Ground

Molex PicoBlade 1.25mm
	Cable P/N 0151340501 (100mm)
	Connector P/N 532610571

## Interface

Management protocol TODO

## Sensors

AT30TS74 temperature sensor (at least one, possibly two)

AD8218 current shunt monitor on input and output rails

Internal STM32L031 ADC is 12 bits, Vref 1.65 to 3.6V
Say we go with 2.5V, 12 bits gives 0.61 mV/LSB

### Input shunt

25 milliohms gives 500 mV/A, so 805 mV full scale range. At 1.61A will dissipate 80.5 mW

With 0.61 mV LSB this gives 1.22 mA resolution (without oversampling)

### Input voltage divider

30:1 divider gives 1.6V nominal, 18.3 mV resolution

### Output shunt

5 milliohms gives 100 mV/A so 800 mV full scale range. At 8A will dissipate 320 mW

With 0.61 mV LSB this gives 6.1 mA resolution (without oversampling)

### Output voltage divider

5:1 divider gives 2.4V nominal, 3.05 mV resolution

# General design notes

Cout 200 to 1200 uF. Put 330 uF in the module

STM32L031 (package TBD)

Soft start load switch on the main 12V rail, driving into additional bulk cap
