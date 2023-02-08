# adcmxl-spi-python
A way to communicate in Realtime Mode with the ADCMXL3032 MEMS Sensor via Python with a Raspberry Pi 4.


# Setup
Keep in mind, that dependencies need to be installed:
- spidev
- numpy 
a.s.o.

The connection between the Pi and the MEMS Sensor should be diagnosed with a suitable Logic Analyzer.
We used the AZDelivery Logic Analyzer which comes with a sampling rate of 20MHz. Keep in mind, that this can cause aliasing when running the SPI interface at the full speed of the ADCMXL3032. 
![alt text]([http://url/to/img.png](https://s3.us-west-2.amazonaws.com/secure.notion-static.com/489fa01b-1d19-468f-a1d0-bd32be35f0be/busypin.png?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Content-Sha256=UNSIGNED-PAYLOAD&X-Amz-Credential=AKIAT73L2G45EIPT3X45%2F20230208%2Fus-west-2%2Fs3%2Faws4_request&X-Amz-Date=20230208T080602Z&X-Amz-Expires=86400&X-Amz-Signature=7f478fcb03ccd6d76ed92e2c8997890a196a1d8046e5d3893c3740803aef449f&X-Amz-SignedHeaders=host&response-content-disposition=filename%3D%22busypin.PNG.png%22&x-id=GetObject))


# Wiring Diagram
The Wiring diagram can be seen below. This uses the Main SPIdev Interface of the Rapsberry Pi. 
[Include Image here]

# Thanks
Many thanks to the person, suggestion their version for the ADCMXL3032 in this forum entry:
https://www.raspberrypi.com/products/raspberry-pi-4-model-b/

