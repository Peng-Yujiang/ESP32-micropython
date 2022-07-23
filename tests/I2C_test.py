import sys
sys.path.append("../drivers") # include driver path
sys.path.append("../resources") # include resources path

### import build-in libs
from machine import SPI, I2C, TouchPad, Pin
import time
import math

### import driver
from bh1750 import BH1750    # light sensor
from imu import MPU6050      # MPU6050
from mpu9250 import MPU9250  # MPU9250
from fusion import Fusion    # Fusion data from IMU
from ssd1306 import SSD1306_I2C  # 0.96 inch oled display
#from st7735 import TFT       # 1.8 inch TFT display
from sysfont import sysfont  # font for display

############################# General board control ###########################
########## machine module ##########
import machine
machine.freq(240000000)    # set the CPU frequency to 240 MHz
CPU_freq = machine.freq()  # current frequency of the CPU
print('The frequency of CPU is: {}MHz'.format(CPU_freq/1e6))

########## esp module ##########
import esp
esp.osdebug(None)   # turn off vendor o/s debugging messages
esp.osdebug(0)      # redirect vendor o/s debugging messages to UART(0)
            # LOG_NONE, LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG, LOG_VERBOSE
### low level methods to interact with flash storage
# esp.flash_size()
# esp.flash_user_start()
# esp.flash_erase(sector_no)
# esp.flash_write(byte_offset, buffer)
# esp.flash_read(byte_offset, buffer)

########## esp32 module ##########
import esp32
### Functions
hall = esp32.hall_sensor()     # read the internal hall sensor
temp = esp32.raw_temperature() # read the internal temperature of the MCU, in Fahrenheit
#esp32.ULP()                   # access to the Ultra-Low-Power Co-processor
print('Magnetic field intensity: {}μT'.format(hall))
print('Temperature of CPU: {:.2f}℃\n'.format((temp-32)/1.8))
# esp32.wake_on_touch(True)   # Wake the device from sleep
# esp32.wake_on_ext0(pin, level) # how EXT0 wakes the device from sleep
#                                # level: esp32.WAKEUP_ALL_LOW or esp32.WAKEUP_ANY_HIGH
# esp32.wake_on_ext1(pin, level) # how EXT1 wakes the device from sleep
# esp32.gpio_deep_sleep_hold(True) # whether non-RTC GPIO pin configuration is retained during deep-sleep mode for held pads
### Flash partitions, includes methods to enable over-the-air (OTA) updates
# 

########## Networking, network module ##########
# wlan.scan()                   # scan for access points
# WLAN_SSID = 'TP-Link_ED11'
# WLAN_KEY = '16158184'
# def do_connect():
#     import network
#     wlan = network.WLAN(network.STA_IF)  # create station interface
#     wlan.active(True)             # activate the interface
#     wlan.confi(reconnects = 10)   # 10 reconnect attemps
#     if not wlan.isconnected():    # check if the station if connected to an AP
#         print('Connecting to network...')
#         wlan.connect(WLAN_SSID, WLAN_KEY) # connnect to an AP
#         while not wlan.isconnected():
#             pass
#     print('Mac:,', wlan.config('mac'))        # get the interface's MAC address
#     print('Network config:', wlan.ifconfig()) # get the interface's IP/netmask/gw/DNS addresses
# """the socket module can be used to create and use TCP/UDP sockets as usual,
# and the urequests module for convenient HTTP requests."""

# work as ap
# ap = network.WLAN(network.AP_IF) # create access-point interface
# ap.config(ssid='ESP-AP') # set the SSID of the access point
# ap.config(max_clients=10) # set how many clients can connect to the network
# ap.active(True)         # activate the interface

########## Delay and timing, time module ##########
# import time

# time.sleep(1)           # sleep for 1 second
# time.sleep_ms(500)      # sleep for 500 milliseconds
# time.sleep_us(10)       # sleep for 10 microseconds
# start = time.ticks_ms() # get millisecond counter
# delta = time.ticks_diff(time.ticks_ms(), start) # compute time difference

########## Timers, timer module ##########
# from machine import Timer
# #The period is in milliseconds, with a timer ID from 0 to 3
# tim0 = Timer(0)
# tim0.init(period=5000, mode=Timer.ONE_SHOT, callback=lambda t:print(0))
# tim1 = Timer(1)
# tim1.init(period=2000, mode=Timer.PERIODIC, callback=lambda t:print(1))

########## Pins and GPIO, machine.Pin ##########
"""Available Pins are from the following ranges (inclusive): 0-19, 21-23, 25-27, 32-39.
   Pins 1 and 3 are REPL UART TX and RX respectively
   Pins 6, 7, 8, 11, 16, and 17 are used for connecting the embedded flash,
       and are not recommended for other uses
   Pins 34-39 are input only, and also do not have internal pull-up resistors"""
# p0 = Pin(0, Pin.OUT)    # create output pin on GPIO0
# p0.on()                 # set pin to "on" (high) level
# p0.off()                # set pin to "off" (low) level
# p0.value(1)             # set pin to on/high
# 
# p2 = Pin(2, Pin.IN)     # create input pin on GPIO2
# print(p2.value())       # get value, 0 or 1
# 
# p4 = Pin(4, Pin.IN, Pin.PULL_UP) # enable internal pull-up resistor
# p5 = Pin(5, Pin.OUT, value=1) # set pin high on creation
# p6 = Pin(6, Pin.OUT, drive=Pin.DRIVE_3) # set maximum drive strength
#     """Pin.DRIVE_0: 5mA / 130 ohm
#        Pin.DRIVE_1: 10mA / 60 ohm
#        Pin.DRIVE_2: 20mA / 30 ohm (default strength if not configured)
#        Pin.DRIVE_3: 40mA / 15 ohm"""
#     """hold= True: the pin configuration (direction, pull resistors and output value)
#            will be held and any further changes (including changing the output level)
#            will not be applied
#        hold=False: immediately apply any outstanding pin configuration changes and
#            release the pin.
#        hold=True while a pin is already held will apply any configuration changes and
#            then immediately reapply the hold."""

left_Button = Pin(13, Pin.IN, Pin.PULL_UP)
right_Button = Pin(15, Pin.IN, Pin.PULL_UP)

def button(button):
    """Interrupt of button pressing"""
    if button == 'Left':
        which_button = left_Button
        pin = 13
    elif button == 'Right':
        which_button = right_Button
        pin = 15
    which_button.irq(trigger=Pin.IRQ_FALLING, handler=lambda pin: _button_callback(button))
            #priority=1, wake=None, hard=False)
            # Priority: higher values always represent higher priorities
            # Wake: machine.IDLE, machine.SLEEP or machine.DEEPSLEEP. use OR for more power mode
            # hard: if true a hardware interrupt is used. This reduces the delay between
            #       the pin change and the handler being called. 
def _button_callback(button):
    """Callback function of button()."""
    print(button,'button pressed.')
    return True
button('Right')
button('Left')


########## UART (serial bus) ##########
# from machine import UART
# uart1 = UART(1, baudrate=9600, tx=33, rx=32) #three hardware UARTs: UART0, UART1 and UART2
# uart1.write('hello')  # write 5 bytes
# uart1.read(5)         # read up to 5 bytes
#     """The default pins for hardware UARTs
#             UART0 UART1 UART2
#         tx    1    10    17
#         rx    3    9     16         """

########## PWM (pulse width modulation) ##########
# The base frequency can range from 1Hz to 40MHz but there is a tradeoff;
#  as the base frequency increases the duty resolution decreases.
# from machine import Pin, PWM
# 
# pwm0 = PWM(Pin(0))         # create PWM object from a pin
# freq = pwm0.freq()         # get current frequency (default 5kHz)
# pwm0.freq(1000)            # set PWM frequency from 1Hz to 40MHz
# 
# duty = pwm0.duty()         # get current duty cycle, range 0-1023 (default 512, 50%)
# pwm0.duty(256)             # set duty cycle from 0 to 1023 as a ratio duty/1023, (now 25%)
# 
# duty_u16 = pwm0.duty_u16() # get current duty cycle, range 0-65535
# pwm0.duty_u16(2**16*3//4)  # set duty cycle from 0 to 65535 as a ratio duty_u16/65535, (now 75%)
# 
# duty_ns = pwm0.duty_ns()   # get current pulse width in ns
# pwm0.duty_ns(250_000)      # set pulse width in nanoseconds from 0 to 1_000_000_000/freq, (now 25%)
# 
# pwm0.deinit()              # turn off PWM on the pin
# 
# pwm2 = PWM(Pin(2), freq=20000, duty=512)  # create and configure in one go
# print(pwm2)                               # view PWM settings
# 
# while True:
# 	from machine import Pin, PWM
# 	import time
# 	for i in range(256):
# 		pwm = PWM(Pin(23), freq = 20000, duty = 4*i)
# 		time.sleep_ms(100)
# 		print(pwm)

########## ADC (analog to digital conversion) ##########
# from machine import ADC, Pin
# 
# adc = ADC(Pin(12))         # create an ADC object acting on a pin
# val = adc.read_u16()  # read a raw analog value in the range 0-65535
# val = adc.read_uv()   # read an analog value in microvolts

########## Real time clock (RTC) ##########
# from machine import RTC
# 
# rtc = RTC()
# rtc.datetime((2017, 8, 23, 1, 12, 48, 0, 0)) # set a specific date and time
# rtc.datetime() # get date and time

########## WDT (Watchdog timer) ##########
# from machine import WDT
# 
# # enable the WDT with a timeout of 5s (1s is the minimum)
# wdt = WDT(timeout=5000)
# wdt.feed()

########## Deep-sleep mode ##########
# import machine
# # check if the device woke from a deep sleep
# if machine.reset_cause() == machine.DEEPSLEEP_RESET:
#     print('woke from a deep sleep')
# # put the device to sleep for 10 seconds
# machine.deepsleep(10000)
# 
# # disable pull resistors to save power before entering deep-sleep mode
# from machine import Pin, deepsleep
# # configure input RTC pin with pull-up on boot
# pin = Pin(2, Pin.IN, Pin.PULL_UP)
# # disable pull-up and put the device to sleep for 10 seconds
# pin.init(pull=None)
# machine.deepsleep(10000)
# 
# from machine import Pin, deepsleep
# import esp32
# opin = Pin(19, Pin.OUT, value=1, hold=True) # hold output level
# ipin = Pin(21, Pin.IN, Pin.PULL_UP, hold=True) # hold pull-up
# # enable pad hold in deep-sleep for non-RTC GPIO
# esp32.gpio_deep_sleep_hold(True)
# # put the device to sleep for 10 seconds
# deepsleep(10000)

########## SD card ##########
# import machine, os
# 
# # Slot 2 uses pins sck=18, cs=5, miso=19, mosi=23
# sd = machine.SDCard(slot=2)
# os.mount(sd, "/sd")  # mount
# os.listdir('/sd')    # list directory contents
# os.umount('/sd')     # eject

########## RMT ##########
# import esp32
# from machine import Pin
# # allows generation of accurate digital pulses with 12.5ns resolution
# r = esp32.RMT(0, pin=Pin(18), clock_div=8)
# r   # RMT(channel=0, pin=18, source_freq=80000000, clock_div=8)
# # The channel resolution is 100ns (1/(source_freq/clock_div)).
# r.write_pulses((1, 20, 2, 40), 0) # Send 0 for 100ns, 1 for 2000ns, 0 for 200ns, 1 for 4000ns

########## OneWire driver ##########
# from machine import Pin
# import onewire
# # The OneWire driver is implemented in software and works on all pins
# # Be sure to put a 4.7k pull-up resistor on the data line
# ow = onewire.OneWire(Pin(12)) # create a OneWire bus on GPIO12
# ow.scan()               # return a list of devices on the bus
# ow.reset()              # reset the bus
# ow.readbyte()           # read a byte
# ow.writebyte(0x12)      # write a byte on the bus
# ow.write('123')         # write bytes on the bus
# ow.select_rom(b'12345678') # select a specific device by its ROM code

########## Capacitive touch ##########
# from machine import TouchPad, Pin
# # There are ten capacitive touch-enabled pins: 0, 2, 4, 12, 13 14, 15, 27, 32, 33.
# t = TouchPad(Pin(14))
# t.read()              # Returns a smaller number when touched
# 
# # TouchPads can be used to wake an ESP32 from sleep
# import machine
# from machine import TouchPad, Pin
# import esp32
# t = TouchPad(Pin(14))
# t.config(500)               # configure the threshold at which the pin is considered touched
# esp32.wake_on_touch(True)
# machine.lightsleep()        # put the MCU to sleep until a touchpad is touched


###################################### I2C ############################################
########## Hardware I2C bus ##########
# from machine import Pin, I2C
# # Any available output-capable pins can be used for SCL and SDA
# i2c = I2C(0) # default scl=Pin(18), sda=Pin(19)
# i2c = I2C(1, scl=Pin(25), sda=Pin(26), freq=400000)

i2c = I2C(0, scl=Pin(18), sda=Pin(19),freq=400000)
i2c_disp = I2C(1, scl=Pin(22), sda=Pin(21),freq=400000)
  # create I2C peripheral at frequency of 400kHz
  # depending on the port, extra parameters may be required
  # to select the peripheral and/or pins to use
addr_disp = i2c_disp.scan()    # scan for peripherals, returning a list of 7-bit addresses
for address in addr_disp:
    i = addr_disp.index(address) + 1
    print('I2C display address {}: {}'.format(i, hex(address)))
addresses = i2c.scan()    # scan for peripherals, returning a list of 7-bit addresses
for address in addresses:
    i = addresses.index(address) + 1
    print('I2C address {}: {}'.format(i, hex(address)))

######### Light sensor ##########
# configure light sensor
light_sensor = BH1750(i2c)  # 0x23
#light_sensor.on()    # Turn sensor on
#light_sensor.reset() # Reset sensor, turn on first if required.
#light_sensor.set_mode(mode)  # Set sensor mode
    # modes
    #CONT_LOWRES = 0x13
    #CONT_HIRES_1 = 0x10
    #CONT_HIRES_2 = 0x11
    #ONCE_HIRES_1 = 0x20
    #ONCE_HIRES_2 = 0x21
    #ONCE_LOWRES = 0x23
luminance = light_sensor.luminance(BH1750.ONCE_HIRES_1)
#light_sensor.off()   # Turn sensor off
print('\nLuminance:{:5.2f} lx\n'.format(luminance))

############ OLED ###########    
# configure 0.96 inch OLED display
OLED_WIDTH = 128
OLED_HEIGHT = 64
oled = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c_disp) # 0x78
oled.rotate(2) # rotate 180 degrees

############ IMU ############
#### configure MPU6050
imu = MPU6050(i2c, 0) # 0x68
print('MPU6050 connected')
fuse = Fusion()
print('Accecerator: ', imu.accel.xyz)
print('Gyroscope: ', imu.gyro.xyz)
print('Temperature in MPU6050: {:.2f}℃\n'.format(imu.temperature))

#### configure MPU9250
# imu = MPU9250(i2c, 1) # 0x0c
# print('MPU9250 connected')
# fuse = Fusion()
# count = 0
# # Code for external switch
# switch = Pin(34, Pin.IN, pull=Pin.PULL_UP) # Switch to ground on Y7
# def sw():
#     return not switch.value()
# 
# # Choose test to run
# Calibrate = True
# Timing = True
# 
# def getmag():                               # Return (x, y, z) tuple (blocking read)
#     return imu.mag.xyz
# 
# if Calibrate:
#     print("Calibrating. Press switch when done.")
#     fuse.calibrate(getmag, sw, lambda : time.sleep_ms(1000))
#     print("Bias of magnetometer:", fuse.magbias)
# 
# ###Choose test to run
# Timing = True
# if Timing:
#     accel = imu.accel.xyz
#     gyro = imu.gyro.xyz
#     magn = imu.mag.xyz
#     start = time.ticks_us()  # Measure computation time only
#     fuse.update(accel, gyro, magn) # 979μs on Pyboard
#     t = time.ticks_diff(time.ticks_us(), start)
#     print("Update time (uS):", t)
# 
# print('Accecerator: ', imu.accel.xyz)
# print('Gyroscope: ', imu.gyro.xyz)
# print('Magnetometer: ', imu.mag.xyz)
# print('Temperature in MPU9250: {:.2f}℃\n'.format(imu.temperature))


###################################### SPI ############################################
########## Hardware SPI bus ##########
# from machine import Pin, SPI
from st7735 import TFT       # 1.8 inch TFT display
# hspi = SPI(1, 10000000)
# hspi = SPI(1, 10000000, sck=Pin(14), mosi=Pin(13), miso=Pin(12))
# vspi = SPI(2, baudrate=80000000, polarity=0, phase=0, bits=8, firstbit=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))


############################# Self-defined functions ##################################
# FONT_SIZE = 8
# GAP = 1
# AXISES = ['X', 'Y', 'Z']
# ORIENTATION = ['Heading', 'Pitch', 'Roll']
# 
# MOTIOM_THRESHOLD = 4
# def display_on_oled(info_str = 'Text', posi_x = 0, posi_y = 0):
#     """Display informaion on oled"""
#     oled.fill(0)
#     oled.text(info_str, posi_x, posi_y)
#     oled.show()
# 
# def detect_motion(threshold = MOTIOM_THRESHOLD, axis = 'xyz'):
#     """Return True if motion detected."""
#     gyro_abs_sum = 0
#     if axis == 'xyz':
#         for i in range(3):
#             gyro_abs_sum += math.fabs(imu.gyro.xyz[i])
#     elif axis == 'x':
#         gyro_abs_sum = math.fabs(imu.gyro.x)
#     elif axis == 'y':
#         gyro_abs_sum = math.fabs(imu.gyro.y)
#     elif axis == 'z':
#         gyro_abs_sum = math.fabs(imu.gyro.z)
#     else:
#         raise ValueError('axis should be x, y, z, or, xyz')
#     print('gyro_abs_sum ', gyro_abs_sum)
#     return gyro_abs_sum > threshold
# 
# HOLD_TIME = 10 # sleep after specified time
# working = True
# last_status = False
# rest_time = 0.0
# sleep_time = 0.0
# while True:
#     start = time.time_ns()  # record time
#     # sleep mode
#     if working:
#         if not detect_motion():
#             deltaT = (time.time_ns() - start)
#             rest_time += deltaT/ 1e8
#             display_on_oled('Rest time: '+str(rest_time), 0, 30)
#         else:
#             rest_time = 0
#             display_on_oled('Moving', 0, 30)
#             time.sleep_ms(1000)
#         if rest_time > HOLD_TIME: # at rest for more than 10 seconds, then sleep
#             imu.sleep
#             oled.fill(0)
#             oled.text('Sleeping', 0, 30)
#             oled.show()
#             last_status = working
#             working = False
#             rest_time = 0
#             work_time = 0
#     elif not working:  # if not working
#         if detect_motion():   # if motion detected, then wake up
#             imu.wake
#             display_on_oled('Awake now')
#             last_status = working
#             working = True
#             sleep_time = 0
#             rest_time = 0
#             time.sleep_ms(1000)
#         
#     time.sleep_ms(1)

        
    #last_working_status = working_status
    
    
# #     fuse.update(imu.accel.xyz, imu.gyro.xyz, imu.mag.xyz)
# #     positions = [fuse.heading, fuse.pitch, fuse.roll]
# #     fuse_6.update_nomag(imu_6.accel.xyz, imu_6.gyro.xyz) # MOU6050
# #     positions_6 = [fuse_6.heading, fuse_6.pitch, fuse_6.roll]
#     oled.fill(0)
# #     for i in range(3):
# # 		oled.text('{}:{:8.3f}'.format(ORIENTATION[i], positions[i]), 0, 0 if i==0 else (FONT_SIZE + GAP)*i)
# # 		oled.text('{}:{:8.3f}'.format(ORIENTATION[i], positions_6[i]), 0, 0 if i==0 else (FONT_SIZE + GAP)*(i+3))
#     #print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(positions[0], positions[1], positions[2]))
#         
#     ### display accel and gyro
#     for i in range(3):
#         oled.text('Acce {}:{:8.3f}'.format(AXISES[i], imu.accel.xyz[i]), 0, 0 if i==0 else (FONT_SIZE + GAP)*i)
#         oled.text('Gyro {}:{:8.3f}'.format(AXISES[i], imu.gyro.xyz[i]), 0, (FONT_SIZE + GAP)*(i+3))
#     oled.text('Temp:{:5.2f}'.format(imu.temperature), 0, (FONT_SIZE + GAP)*6)
#     
#     ### display light intensity
#     #oled.text('Light:{:5.2f} lx'.format(light_sensor.luminance(BH1750.ONCE_HIRES_1), 0, (FONT_SIZE + GAP)*6)
#     oled.show()
#     #time.sleep_ms(1)

