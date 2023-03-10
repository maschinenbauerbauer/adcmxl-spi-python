# adcmxl3021 Driver in Python
# SPI Master is using Raspberry Pi
# spidev is the Raspberry Pi spi communication library
# Date: 2019.7.11

from gettext import npgettext
import numpy as np
from collections import deque

import sys
import os
import spidev
import time
import RPi.GPIO as GPIO_SPI

SPI_MAX_SPEED   = int(15.625*10**6)#=1953125

r_pin = 26
y_pin = 19
g_pin = 13

class adcmxl3021():
    
    #Configuration Constant based on ADcmXL3021 datasheet
    
    PAGE_ID         =0x00
    TEMP_OUT        =0x02
    SUPPLY_OUT      =0x04
    FFT_AVG1        =0x06
    FFT_AVG2        =0x08
    BUF_PNTR        =0x0A
    REC_PNTR        =0x0C
    X_BUF           =0x0E
    Y_BUF           =0x10
    Z_BUF           =0x12
    X_ANULL         =0x14
    Y_ANULL         =0x16
    Z_ANULL         =0x18
    REC_CTRL        =0x1A
    RT_CTRL         =0x1C
    REC_PRD         =0x1E
    ALM_F_LOW       =0x20
    ALM_F_HIGH      =0x22    
    ALM_X_MAG1      =0x24   
    ALM_Y_MAG1      =0x26
    ALM_Z_MAG1      =0x28   
    ALM_X_MAG2      =0x2A   
    ALM_Y_MAG2      =0x2C
    ALM_Z_MAG2      =0x2E
    ALM_PNTR        =0x30
    ALM_S_MAG       =0x32
    ALM_CTRL        =0x34
    DIO_CTRL        =0x36
    FILT_CTRL       =0x38
    AVG_CNT         =0x3A
    DIAG_STAT       =0x3C
    GLOB_CMD        =0x3E
    
    ALM_X_STAT      =0x40   
    ALM_Y_STAT      =0x42
    ALM_Z_STAT      =0x44   
    ALM_X_PEAK      =0x46   
    ALM_Y_PEAK      =0x48
    ALM_Z_PEAK      =0x4A

    TIME_STAMP_L    =0x4C
    TIME_STAMP_H    =0x4E

    REV_DAY         =0x52
    YEAR_MON        =0x54
    PROD_ID         =0x56

    SERIAL_NUM      =0x58
    USER_SCRATCH    =0x5A
    REC_FLASH_CNT   =0x5C

    MISC_CTRL       =0x64
    REC_INFO1       =0x66
    REC_INFO2       =0x68
    REC_PNTR        =0x6A

    ALM_X_FREQ      =0x6C
    ALM_Y_FREQ      =0x6E   
    ALM_Z_FREQ      =0x70
    STAT_PNTR       =0x72
    X_STASTIC       =0x74
    Y_STASTIC       =0x76
    Z_STASTIC       =0x78
    FUND_FREQ       =0x7A
    FLASH_CNT_L     =0x7C
    FLASH_CNT_H     =0x7E

    PAGE_ID         =0x00

    def __init__(self, device=0, ce_pin=0):
        """
        device: the SPI device (often 0)
        ce_pin: pass 0 for CE0, 1 for CE1, etc.
        device and ce_pin map to device file /dev/spidev{device}.{ce_pin}
        """
        self.spi_init(device, ce_pin)

    def spi_init(self,device,ce_pin):
        """
        Using "pinout" command to check the raspberry spi definition.
        device: SPI0 (0), SPI1 (1)
        ce_pin: CE0 (0), CE1 (1)
        
        SPI0 definition
        CE0 :  GPIO8 (0)  PIN24
        CE1 :  GPIO7 (1)  PIN26
        MOSI:  GPIO10     PIN19
        MISO:  GPIO9      PIN21
        SCLK:  GPIO11     PIN23

        SPI1 definition
        CE0 :  GPIO18 (0) PIN12 
        CE1 :  GPIO17 (1) PIN11PAGE_ID
        CE2 :  GPIO16 (2) PIN36
        MOSI:  GPIO20     PIN38
        MISO:  GPIO19     PIN35
        SCLK:  GPIO21     PIN40   
        """
        # init spi for communication
        self.spi = spidev.SpiDev()
        self.spi.open(device, ce_pin) # (x,0) == CE0, (x,1) == CE1

        # Set 4-line spi mode
        self.spi.threewire   =False

        # Set spi msb first
        self.spi.lsbfirst    =False

        # Set clock phase and polarity to default
        self.spi.mode       =3

        # Set SPI CS Active Low
        #self.spi.cshigh     =False

        # Set SPI clock rate: 7.8125MHz, cdiv=32, fsclk=250MHz/cdiv, cdiv = [2,4,8,16,32,...32768]
        self.spi.max_speed_hz=SPI_MAX_SPEED

        self.spi.bits_per_word =8

        # GPIO Setup
        GPIO_SPI.setmode(GPIO_SPI.BCM)

        # Busy Pin Setup
        GPIO_SPI.setup(22, GPIO_SPI.IN, pull_up_down=GPIO_SPI.PUD_UP)

        # LED Setup
        GPIO_SPI.setup(r_pin, GPIO_SPI.OUT)
        GPIO_SPI.output(r_pin, True)
        GPIO_SPI.setup(y_pin, GPIO_SPI.OUT)
        GPIO_SPI.output(y_pin, True)
        GPIO_SPI.setup(g_pin, GPIO_SPI.OUT)
        GPIO_SPI.output(g_pin, True)
        #time.sleep(2) # this makes sure, that the spi communication starts after the booting process
        #GPIO_SPI.output(r_pin, False)
        GPIO_SPI.output(y_pin, False)
        GPIO_SPI.output(g_pin, False)


    def spi_write(self, address, value):
        # Write value (1 byte) to address
        # Send instruction (command bit7=1 for write), address[6:0], and value
        address=address+0x80
        self.spi.xfer([address, value])

    def spi_write_word(self,address,value_16b):
        # This method is needed because the communication protocol of the chip
        # only supports 8-bit data transfer but the registers hold 16-bit.
        # Use this to write to registers
        value_l=value_16b & 0xFF
        value_h=value_16b>>8
        print('{}: {}'.format(address, hex(value_16b)))
        self.spi_write(address,value_16b)
        self.spi_write(address+1,value_h)
        # Dummy read (the first read is always faulty)
        self.spi_read_bytes(address)

    def spi_read_bytes(self, address):
        # Read
        # Send instruction (command bit7=0 for read), address bit6:0
        rb=self.spi.xfer([address,0],int(12.5*10**6),16,8)
        return rb

    def spi_read_rts(self, address, delay_sec):
        # Read
        # Send instruction (command bit7=0 for read), address bit6:0
        delay = delay_sec*10**6
        rb=self.spi.xfer([address,0],int(12.5*10**6),delay,8)
        print(rb)
        rbc=  rb[0]*256 + rb[1]
        return rbc

    def spi_read(self, address):
        # Read
        # Send instruction (command bit7=0 for read), address bit6:0
        rb=self.spi.xfer([address,0],int(12.5*10**6),16,8)
        rbc=  rb[0]*256 + rb[1]
        return rbc
        
    def spi_read2s(self, address):
        # Read
        # Send instruction (command bit7=0 for read), address bit6:0
        rb=self.spi.xfer([address,0],int(12.5*10**6),16,8)
        rbc=  rb[0]*256 + rb[1]
        rbc2= self.twos_comp(rbc,16)
        return rbc2

    def twos_comp(self,val, bits):
        # two's complement of value in bits
        if val&(1<<(bits-1)) != 0:
            val = val - (1<<bits)
        return val

    def check_spi_rd(self):
        #Readback Device ID to check the SPI Read
        self.get_prod_id()
        rb=self.get_prod_id()
        print ("--------------")
        print ("product_id readback:%s"%(rb))
        if (rb == 3021) :
            print ("spi Read OK, Find DUT=ADcmXL3021")
            return 1
        else:
            GPIO_SPI.output(r_pin, True)
            time.sleep(1)
            GPIO_SPI.output(r_pin, False)
            time.sleep(1)
            GPIO_SPI.output(r_pin, True)
            time.sleep(1)
            print ("spi Read Failure, Please check SPI timing and hardware connections")
            return 0

    def software_reset(self):
        self.spi_read(self.GLOB_CMD)
        rb=self.spi_read(self.GLOB_CMD)
        rb=rb | 0x80
        self.spi_write_word(self.GLOB_CMD,rb)
        #wait s
        time.sleep(0.5)

    def get_diag_sts(self):
        self.spi_read(self.DIAG_STS)
        rb=self.spi_read(self.DIAG_STS)
        return hex(rb)

    def get_temp_out(self):
        #return unit:C
        #self.set_page_id(page_id=0)
        self.spi_read(self.TEMP_OUT)
        rb=self.spi_read(self.TEMP_OUT)
        return (self.twos_comp(rb,16)*(-0.46)+460)   

    def get_prod_id(self):
        #self.set_page_id(page_id=0)
        self.spi_read(self.PROD_ID)
        rb=self.spi_read(self.PROD_ID)
        return rb

    def set_user_scratch(self,user_scratch):
        #self.set_page_id(page_id=0)
        self.spi_write_word(self.USER_SCRATCH,user_scratch)

    def get_user_scratch(self):
        #self.set_page_id(page_id=0)
        self.spi_read(self.USER_SCRATCH)
        rb=self.spi_read(self.USER_SCRATCH)
        return rb

    def get_year_mon_day_rev(self):
        #self.set_page_id(page_id=0)
        self.spi_read(self.YEAR_MON)
        year_mon=self.spi_read(self.YEAR_MON)
        self.spi_read(self.REV_DAY)
        rev_day=self.spi_read(self.REV_DAY)
        return year_mon, rev_day


    def set_page_id(self,page_id):
        self.spi_write_word(self.PAGE_ID,page_id)

    def get_page_id(self):
        self.spi_read(self.PAGE_ID)
        rb=self.spi_read(self.PAGE_ID)
        return rb

    def record_start(self):
        self.spi_read(self.GLOB_CMD)
        rb=self.spi_read(self.GLOB_CMD)
        print(rb)
        rb=rb | 0x0800 # Set Bit 11 to 1, see  Table 91 in documentation
        time.sleep(.5)
        self.spi_write_word(self.GLOB_CMD,rb)

    def start_rts_mode(self, id, pnr, supplier, period, date):
        ### Set REC_CTRL ###
        val = 0x9103 # this sets the software-rec trigger mode and the real time streaming mode
        print('Value to write: {}'.format(bin(val)))
        self.spi_write_word(self.REC_CTRL, val)
        time.sleep(1)
        read = self.spi_read_bytes(self.REC_CTRL)
        print(read)
        print('After: high_byte: {} low_byte:{}'.format(bin(read[0]), bin(read[1])))


        ### SET RT_CTRL ###
        val = 0x0081 # this sets the sample rate to 40kHz
        print('Value to write: {}'.format(bin(val)))
        self.spi_write_word(self.RT_CTRL, val)
        time.sleep(1)
        read = self.spi_read_bytes(self.RT_CTRL)
        print(read)
        print('After: high_byte: {} low_byte:{}'.format(bin(read[0]), bin(read[1])))

        time.sleep(5) # necessary for the autostart procedure. Otherwise data loss

        data = [bytearray(0)]*625000
        idx_counter = 0

        seconds = 24
        ns = seconds * 10**9
        speed_hz = int(12.5*10**6)
        sleep_delay = .10*10**-3 # in s 0.0001s

        null_array = bytearray(200)

        GPIO_SPI.output(y_pin, True)
        GPIO_SPI.output(r_pin, False)
        try:
            GPIO_SPI.add_event_detect(22, GPIO_SPI.RISING) # Busy Pin event detection
            self.record_start()
            #time.sleep(.012) # not necessary because of rising edge detection

            start_time_ns = int(time.time_ns()) # epoch unix time

            print('Started recording...')

            while int(time.time_ns()) - start_time_ns <= ns:   
                    if GPIO_SPI.event_detected(22):
                        data[idx_counter] = self.spi.xfer3(null_array, speed_hz, 0)
                        #data[idx_counter] = [i for i in data[idx_counter]]
                        #data[idx_counter].append(int(time.time_ns()))
                        idx_counter += 1
                        #print('Transaction {}'.format(idx_counter))
                        


            print('Recording finished.')
            GPIO_SPI.output(g_pin, True)

            data = data[:idx_counter]

            data = np.array(data).ravel()

            print('Writing data...')
            try:
                # Save to first mounted usb device
                devices = os.listdir('/media/logger-pi/')
                filename = '/media/logger-pi/{}/{}_{}_{}_{}_{}_{}_{}s_{}MHz.txt'.format(devices[0] ,int(time.time()), date, id, pnr.strip(), supplier.strip(), period.replace('/', ''), seconds, speed_hz)
                np.savetxt(
                    filename, 
                    data, 
                    delimiter=','
                )
                filesize = os.path.getsize(filename)
            except:
                # Save to folder on sd card
                filename = '/home/logger-pi/spi-mems-measurements/{}_{}_{}_{}_{}_{}_{}s_{}MHz.txt'.format(int(time.time()), date, id, pnr.strip(), supplier.strip(), period.replace('/', ''), seconds, speed_hz)
                np.savetxt(
                    filename, 
                    data, 
                    delimiter=','
                )
                filesize = os.path.getsize(filename)

            print('Filesize: {} Byte'.format(filesize))
            if filesize < 116000000:
                # this is a faulty measurement because 3.5% were lost
                with open('/home/logger-pi/Schreibtisch/data.txt', 'a') as file:
                    file.write('ERROR\n')

            self.spi.close()
            GPIO_SPI.output(y_pin, False)
        except Exception as e:
            print(e)
            GPIO_SPI.output(r_pin, True)
            time.sleep(1)
            GPIO_SPI.output(r_pin, False)
            time.sleep(1)
            GPIO_SPI.output(r_pin, True)
            time.sleep(1)
            GPIO_SPI.cleanup()
        pass

    def shs_tests(self):
        print('####')
        read = self.spi_read_bytes(self.REC_CTRL)
        # read[0] is low byte, read[1] high byte
        print('Before: high_byte: {} low_byte:{}'.format(bin(read[1]), bin(read[0])))
        val = 0x0010
        print('Value to write: {}'.format(bin(val)))
        self.spi_write_word(self.REC_CTRL, val)
        time.sleep(1)
        read = self.spi_read_bytes(self.REC_CTRL)
        print('After: high_byte: {} low_byte:{}'.format(bin(read[1]), bin(read[0])))



if __name__=="__main__":
    #time.sleep(20) # wait for booting to complete
    sensor = adcmxl3021(device=0, ce_pin=0)
    #sensor.software_reset()
    #time.sleep(2)
    check_ok = sensor.check_spi_rd()
    while not check_ok :
        print('Retrying..')
        sensor = adcmxl3021(device=0, ce_pin=0)
        sensor.software_reset()
        time.sleep(5)
        check_ok = sensor.check_spi_rd()

    data_input = []
    with open('/home/logger-pi/Schreibtisch/data.txt', 'r') as file:
        for line in file.readlines():
            data_input.append(line.strip())

    sensor.start_rts_mode(data_input[0], data_input[1], data_input[2], data_input[3], data_input[4])
    time.sleep(.5)

    

    
