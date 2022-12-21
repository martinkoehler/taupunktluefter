import micropython
import dht
import math
from machine import Pin, I2C, Timer, reset
from time import sleep_ms
from machine_i2c_lcd import I2cLcd
from neopixel import NeoPixel # We have a ws2812rgb LED

# Define constants
RELAIPIN = 6 # Anschluss des Lüfter-Relais

DHTPIN_1 = 2  # Datenleitung für den DHT-Sensor 1 (innen)
DHTPIN_2 = 10 # Datenleitung für den DHT-Sensor 2 (außen)

# RGB Led
PIN_NP = 23
LEDS = 1
BRIGHTNESS = 10

# USR Button
USR_PIN = 13

# *******  Korrekturwerte der einzelnen Sensorwerte  *******
Korrektur_t_1 = -2 # Korrekturwert Innensensor Temperatur
Korrektur_t_2 = -1 # Korrekturwert Außensensor Temperatur
Korrektur_h_1 =  0 # Korrekturwert Innensensor Luftfeuchtigkeit
Korrektur_h_2 = -1 # Korrekturwert Außensensor Luftfeuchtigkeit
#***********************************************************

SCHALTmin = 5.0   # minimaler Taupunktunterschied, bei dem das Relais schaltet
HYSTERESE = 1.0   # Abstand von Ein- und Ausschaltpunkt
TEMP1_min = 10.0  # Minimale Innentemperatur, bei der die Lüftung aktiviert wird
TEMP2_min = -10.0 # Minimale Außentemperatur, bei der die Lüftung aktiviert wird

# Spezielle Zeichen
ue = 245
grad = 223

# Globale Variablen
h1 = 0.0 # Innenluftfeuchtigkeit
t1 = 0.0 # Innentemperatur
t2 = 0.0 # Außentemperatur
h2 = 0.0 # Außenluftfeuchtigkeit

def taupunkt(t, r):
    if (t >= 0):
        a = 7.5
        b = 237.3
    elif (t < 0):
        a = 7.6
        b = 240.7
    else:
        a = b = 0
  
    # Sättigungsdampfdruck in hPa
    sdd = 6.1078 * 10**((a*t)/(b+t))
  
    # Dampfdruck in hPa
    dd = sdd * (r/100)
  
    # v-Parameter
    v = math.log10(dd/6.1078)
  
    # Taupunkttemperatur (°C)
    tt = (b*v) / (a-v)
    return tt

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

class Alarm_timer(Singleton):
    led = Led()
    
    def __init__(self):
        self.heartbeat_ref = self.heartbeat
        self.measure_ref = measure
        self.display_ref = display
        self.timer1= Timer(period=1000, mode=Timer.PERIODIC, callback=self._cb1) # Heartbeat: Jede Sekunde
        self.timer2= Timer(period=10000, mode=Timer.PERIODIC, callback=self._cb2) # Anzeige: Alle 10s
        self.timer3= Timer(period=5000, mode=Timer.PERIODIC, callback=self._cb3) # Messung: Alle 5s
    
    def stop(self):
        self.timer1.deinit()
        self.timer2.deinit()
        self.timer3.deinit()

    # These call backs are interrupt driven, hence complicated functions are not allowed
    # We use micropython.schedule to start the "real" worker
    # We war not allowed to allocate memory in the ISR See
    # https://docs.micropython.org/en/latest/reference/isr_rules.html#isr-rulese
    def _cb1(self, tim):
        micropython.schedule(self.heartbeat_ref, tim)
    def _cb2(self, tim):
        micropython.schedule(self.display_ref, tim)
    def _cb3(self, tim):
        micropython.schedule(self.measure_ref, tim)
    
    def heartbeat(self,args=None):
        self.led.blink()

def measure(args=None):
    global h1, t1, h2, t2
    fehler = False
    msg0 = ""
    msg1 = ""
    try:
        dht1.measure()                           # Sensoren lesen
        h1 = dht1.humidity()+Korrektur_h_1       # Innenluftfeuchtigkeit auslesen und unter „h1“ speichern
        t1 = dht1.temperature()+ Korrektur_t_1   # Innentemperatur auslesen und unter „t1“ speichern
    except OSError as e:
        fehler = True
    if (fehler == True or h1 > 100 or h1 < 1 or t1 < -40 or t1 > 80 ):
        msg0="Fehler Sensor 1"
    elif (firstrun):
        msg0="Sensor 1 ok"
    try:
        dht2.measure()                           # Sensoren lesen
        h2 = dht2.humidity()+Korrektur_h_2       # Außenluftfeuchtigkeit auslesen und unter „h2“ speichern
        t2 = dht2.temperature()+ Korrektur_t_2   # Außentemperatur auslesen und unter „t2“ speichern
    except OSError as e:
        fehler = True
    if (fehler == True or h2 > 100 or h2 < 1 or t2 < -40 or t2 > 80 ):
        msg1="Fehler Sensor 2"
    elif (firstrun):
        msg1="Sensor 2 ok"
    if msg0 or msg1:
        lcd.clear()
        lcd.move_to(0,0)
        lcd.putstr(msg0)
        lcd.move_to(0,1)
        lcd.putstr(msg1)
        sleep_ms(1000)  # Zeit um das Display zu lesen
    if (fehler == True):
        Relais.on() # "High": Lüfter "aus", da bei "low" Lüfter "an"
        lcd.clear()
        lcd.move_to(0,0)
        lcd.putstr("Reset ...")
        sleep_ms(1000)
        reset()
        
def display(args=None):
    # **** Taupunkte errechnen********
    Taupunkt_1 = taupunkt(t1, h1)
    Taupunkt_2 = taupunkt(t2, h2)

    # Werteausgabe auf dem I2C-Display
    """
    Grad Angaben auf ein Grad genau (3 Stellen, da Minusgrade)
    Luftfeuctigkeit auf 1% genau (99% maximal ???)
    Taupunkt auf 0,1 Grad ?
    0123456789ABCDEF
    xxx°C|xx%|xx,x°C
    xxx°C|xx%|xx,x°C
    """
    _t1 = round(t1) # Convert to integer
    _h1 = round(h1) # Convert to integer
    _t2 = round(t2) # Convert to integer
    _h2 = round(h2) # Convert to integer
    # Genaue Ausgabe auf Konsole
    out  = "S1: {:4.3}°C|{:2.2}%|{:4.2f}°C".format(t1,h1,Taupunkt_1)
    out += "\n"
    out += "S2: {:4.3}°C|{:2.2}%|{:4.2f}°C".format(t2,h2,Taupunkt_2)
    print(out)
    lcd.clear()
    lcd.move_to(0,0)
    out1 = ("{:3}"+chr(grad) +"C|{:2}%|{:4.1f}"+chr(grad)+"C").format(_t1,_h1,Taupunkt_1)
    out2 = ("{:3}"+chr(grad)+"C|{:2}%|{:4.1f}"+chr(grad)+"C").format(_t2,_h2,Taupunkt_2)
    lcd.putstr(out1)
    lcd.move_to(0,1)
    lcd.putstr(out2)
    DeltaTP = Taupunkt_1 - Taupunkt_2
    print("DeltaTP {:4.1f}".format(DeltaTP))

    if (DeltaTP > (SCHALTmin + HYSTERESE)):
        rel = True
    if (DeltaTP < (SCHALTmin)):
        rel = False
    if (t1 < TEMP1_min ):
        rel = False
    if (t2 < TEMP2_min ):
        rel = False

    if (rel == True):
        Relais.off() # Relais einschalten
        rgb_led.set(RGB_led.red)
        print("Lüfter ein")
    else:
        Relais.on() # Relais ausschalten
        rgb_led.set(RGB_led.green)
        print("Lüfter aus") 
    
# Setup

dht1 = dht.DHT22(Pin(DHTPIN_1))   # Der Innensensor wird ab jetzt mit dht1 angesprochen
dht2 = dht.DHT22(Pin(DHTPIN_2))   # Der Außensensor wird ab jetzt mit dht2 angesprochen
Relais = Pin(RELAIPIN,Pin.OUT)
Relais.on() # Relais ausschalten
rgb_led = RGB_led()
# Initialisierung I2C
i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=100000)
# Initialisierung LCD über I2C
lcd = I2cLcd(i2c, 0x27, 2, 16) # LCD: I2C-Addresse und Displaygröße setzen
lcd.backlight_on()                      
lcd.move_to(0,0)
lcd.putstr("Teste Sensoren..")
sleep_ms(2000)                    # Time for sensor setup
lcd.clear()
firstrun = True
fehler = measure()
display()
firstrun=False

alarm = Alarm_timer()