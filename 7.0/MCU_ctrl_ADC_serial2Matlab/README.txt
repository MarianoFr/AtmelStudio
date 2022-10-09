"PEER2PEER2_SPI"This script, MCU_ctrl_ADC_serial2Matlab, is the program for the ATmega256RFR2 to implement Ziggy's transmitter.
It is in charge of configuring the external ADC, the MCP3561. After to that, it will give the order to start
conversions, will receive the samples and load them into a buffer.
This script comments all the lightweight mesh functions and functionality, the program's only intention was 
to learn to use the SPI module and design the strategy to configure the ADC MCP3561.
Also the atmega sends via serial port the samples which are received by the PC. We read the samples with Matlab
and reconstruct the audio using the audio toolbox.