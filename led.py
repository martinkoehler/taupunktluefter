from machine import Pin
from neopixel import NeoPixel # We have a ws2812rgb LED
from time import sleep_ms
# RGB Led
PIN_NP = 23
LEDS = 1
BRIGHTNESS = 10

# We use singletons
class Singleton(object):
  def __new__(cls):
    if not hasattr(cls, 'instance'):
      cls.instance = super(Singleton, cls).__new__(cls)
    return cls.instance

class RGB_led(Singleton):
    # GPIO-Pin für WS2812
    pin_np = PIN_NP
    # Anzahl der LEDs
    leds = LEDS
    # Helligkeit: 0 bis 255
    brightness = BRIGHTNESS
    white = (brightness, brightness, brightness)
    red = (brightness, 0, 0)
    green = (0, brightness, 0)
    blue = (0, 0, brightness)
    yellow = (brightness, brightness, 0)
    pink = (brightness, 0, brightness)
    turquoise = (0, brightness, brightness)
    off = (0, 0, 0)
    np = NeoPixel(Pin(pin_np, Pin.OUT), leds)
    def __init__(self):
        self.status = RGB_led.off
        self.np[0] = self.status
        self.np.write()
    
    def set(self,color):
        self.np[0] = color
        self.np.write()
        self.status = color
    
    def blink(self, color, ms=50, num=1):
        for i in range(0,num):
            self.np[0] = color
            self.np.write()
            sleep_ms(ms)
            self.np[0] = self.off
            self.np.write()
            sleep_ms(ms)
        self.np[0] = self.status
        self.np.write()
        
class Led(Singleton):
    def __init__(self):
        # Initialisierung von GPIO25 als Ausgang
        self.led_onboard = Pin(25, Pin.OUT)
        self.led_onboard.off()
        self.status = 0
    
    def on(self):
        self.led_onboard.on()
        self.status = 1
        
    def off(self):
        self.led_onboard.off()
        self.status = 0        
        
    def blink(self, ms=50, num=1):
        for i in range(0,num):
            self.led_onboard.on()
            sleep_ms(ms)
            self.led_onboard.off()
            sleep_ms(ms)
        self.led_onboard.value(self.status)
