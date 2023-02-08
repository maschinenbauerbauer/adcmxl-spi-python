# adcmxl-spi-python
A way to communicate in Realtime Mode with the ADCMXL3032 MEMS Sensor via Python with a Raspberry Pi 4.


# Setup
Keep in mind, that dependencies need to be installed:
- spidev
- numpy 
a.s.o.

The connection between the Pi and the MEMS Sensor should be diagnosed with a suitable Logic Analyzer.
We used the AZDelivery Logic Analyzer which comes with a sampling rate of 20MHz. Keep in mind, that this can cause aliasing when running the SPI interface at the full speed of the ADCMXL3032. 


# Wiring Diagram
The Wiring diagram can be seen below. This uses the Main SPIdev Interface of the Rapsberry Pi. 
[Include Image here]

# Thanks
Many thanks to the person, suggestion their version for the ADCMXL3032 in this forum entry:
https://www.raspberrypi.com/products/raspberry-pi-4-model-b/

