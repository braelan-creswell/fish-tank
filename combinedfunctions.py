import machine, onewire, ds18x20, time, ssd1306, urandom, neopixel, os
from machine import Pin, SoftI2C

os.dupterm(None, 1)

#Temp Sensor Setup
f = 67
ds_pin = machine.Pin(14) #D5 pin is GPIO14
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))

roms = ds_sensor.scan()
print('Found DS devices: ', roms)

#Relay Pins
pump = machine.Pin(12, Pin.OUT) #D6
heater = machine.Pin(16, Pin.OUT) #D0
drain = machine.Pin(13, machine.Pin.OUT) #D7
drainpwm = machine.PWM(drain) #Turns drain pin into a PWM output
drainpwm.freq(1000) #Set control frequency to 500Hz

#NeoPixels
LEDPin = 15 #D8 Pin
NumLeds = 15 #number of pixels
np = neopixel.NeoPixel(machine.Pin(LEDPin), NumLeds)
LEDtime = time.ticks_ms()
LEDdelay = 200
setting = 1

#Float Switches
LowLevel = machine.Pin(0, machine.Pin.IN, machine.Pin.PULL_UP) #D3
HighLevel = machine.Pin(2, machine.Pin.IN, machine.Pin.PULL_UP) #D4

#Button
button = machine.Pin(1, machine.Pin.IN, machine.Pin.PULL_UP) #TX
waterchangeflag = False
adc = machine.ADC(0)

#Screen Pin assignment
i2c = SoftI2C(scl=Pin(5), sda=Pin(4)) #SCL=D1 SDA=D2

oled_width = 128
oled_height = 64
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
screentime = time.ticks_ms()

#Control Variables
SetTemp = 64 #Set Water Temp
PosHysteresis = 0.9
NegHysteresis = 0.2
draindelay = 0

#Functions##########################################################

#Heater Control
timeheat = time.ticks_ms()

def HeaterCtrl():
  global timeheat
  global f
  if time.ticks_ms() >= timeheat + 1000:
    timeheat = time.ticks_ms()
    ds_sensor.convert_temp()
    time.sleep_ms(50)
    for rom in roms:
      f = ds_sensor.read_temp(rom) * (9/5) + 32
      #print(ds_sensor.read_temp(rom))
      #print(f)
      if f >= SetTemp + PosHysteresis:
        heater.value(1)
      elif f <= SetTemp - NegHysteresis:
        heater.value(0)
    

#Pump Control
def AutoPumpCtrl():
  if LowLevel.value() == 1:
    time.sleep_ms(20)
    if LowLevel.value() == 1:
      time.sleep_ms(300)
      pump.value(1)
    else:
      return
  else:
    time.sleep_ms(20)
    if LowLevel.value() == 0:
      pump.value(0)
    else:
      return
      
#Pump Control
def AutoDrainCtrl():
  global draindelay
  if HighLevel.value() == 1:
    time.sleep_ms(20)
    if HighLevel.value() == 1 and draindelay == 0:
      drainpwm.duty(1023)
      time.sleep_ms(100)
      drainpwm.duty(600)
      draindelay = 1
    else:
      return
  else:
    time.sleep_ms(20)
    if HighLevel.value() == 0 and LowLevel.value() == 0:
      drainpwm.duty(0)
      draindelay = 0
    else:
      return

#Water Changes
def WaterChange():
  global waterchangeflag
  if HighLevel.value() == 0:
    pump.value(0)
  else:
    pump.value(1)
    waterchangeflag = False

#Screen Control
def ScreenCtrl():
  global screentime
  if time.ticks_ms() >= screentime + 500:
    screentime = time.ticks_ms()
    oled.fill(0)
    oled.text("Temp: " + "{:.2f}".format(f) + " F", 0, 0)
    oled.text("Heater: On" if heater.value() == 0 else "Heater: Off", 0, 10)
    oled.text("Pump: On" if pump.value() == 0 else "Pump: Off", 0, 20)
    oled.text("Drain: Off" if drainpwm.duty() == 0 else "Drain: On", 0, 30)
    voltage = adc.read()/1024 * 3.3 * 6.384 #Convert voltage from resistor ladder 330k-69k from 0-3.3v scale to 0-19.083v scale
    oled.text("Supply V: " + "{:.2f}".format(voltage), 0, 40)
    oled.text("WC: Off" if waterchangeflag == False else "WC: On", 0, 50)
    
    oled.show()

#Random Number Generation
def randint(min, max):
    span = max - min + 1
    div = 0x3fffffff // span
    offset = urandom.getrandbits(30) // div
    val = min + offset
    return val

#LED Control
def LEDRainbow():
  global LEDtime
  if time.ticks_ms() >= LEDtime + LEDdelay:
    LEDtime = time.ticks_ms()
    for i in range(NumLeds):
      r = randint(0, 255)
      g = randint(0, 255)
      b = randint(0, 255)
      np[i] = (r, g, b)

    np.write()


def LEDCtrl():
  global setting
  if setting == 2:
    LEDRainbow()
  elif setting == 1:
    for i in range(NumLeds):
      np[i] = (255, 255, 255)
  elif setting == 0:
    for i in range(NumLeds):
      np[i] = (0, 0, 0)

  np.write()

#Button Control
def Button():
  global setting
  global waterchangeflag
  if button.value() == 0:
    time.sleep_ms(300)
    if button.value() == 1:
      if setting < 2:
        setting = setting + 1
      else:
        setting = 0
    else:
      time.sleep_ms(300)
      waterchangeflag = not waterchangeflag
 
#Loop##############################################################################

while True:
  AutoDrainCtrl()
  HeaterCtrl()
  ScreenCtrl()
  Button()
  LEDCtrl()
  if waterchangeflag == False:
    AutoPumpCtrl()
  elif waterchangeflag == True:
    WaterChange()







