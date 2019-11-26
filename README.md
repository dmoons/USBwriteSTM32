# USBwriteSTM32

An STM32 processor is used to read data from an ADC (to be replaced with an audio codec of 24 or 32 bit resolution) 
and  transferrs the data to a PC via USB. 

FreeRTOS is used to manage different tasks. All the sampling is done from the interrupt service routine and written
to a buffer, which is transmitted once full. 

## Future of project

Use an audio codec IC for creating a DAC sine wave of 20Hz-20kHZ and then reading the output of an audio amplifier. 
Alternatively a signal source can generate the wave and the output of the amplifier sampled. This will allow the 
computation of total harmonic distortion in the signal, a useful measure for the performance of audio amplfiers. 
