import micropython, dht, math, time, os, io, machine
from machine import Pin, I2C, Timer, reset
from time import sleep_ms
from machine_i2c_lcd import I2cLcd
from led import Singleton, RGB_led, Led

# Define constants
RELAIPIN = 6 # Anschluss des Lüfter-Relais

DHTPIN_1 = 2  # Datenleitung für den DHT-Sensor 1 (innen)
DHTPIN_2 = 10 # Datenleitung für den DHT-Sensor 2 (außen)


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

# Logfile (Einträge alle 5 min, einmal pro Tag geschrieben)
# Date,t1,h1,t2,h2,fan
# 23.08.2022 01:12:48,22.3,40.5,7.6,39.3,1
LOGFILENAME = "taupunkt.csv"

# Spezielle Zeichen
ue = 245
grad = 223

# Globale Variablen
h1 = 0.0 # Innenluftfeuchtigkeit
t1 = 0.0 # Innentemperatur
t2 = 0.0 # Außentemperatur
h2 = 0.0 # Außenluftfeuchtigkeit

logbuffer = io.StringIO()

# Helper functions
# ======================================
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

def pt(t = None):
    """
    Format a time integer in human readable form (pt for Pretty format Time)
    """
    if t == None:
        t = time.time()
        if t > 1913677968 : # Hot fix for emulator
            t = time.mktime((2022, 8, 23, 1, 12, 48, 0, 0))
    y, mm, d, h, m, s = time.localtime(t)[0:6]
    return f"{d:02d}.{mm:02d}.{y} {h:02d}:{m:02d}:{s:02d}"

def print_log(name=LOGFILENAME):
    with open(name,"rt") as f:
        for line in f:
            print(line)

# Hauptklasse
class Alarm_timer(Singleton):
    def __init__(self):
        self.measure_ref = measure
        self.display_ref = display
        self.logdta_ref = logdta
        self.timer1 = Timer(period=2000, mode=Timer.PERIODIC, callback=self._cb1) # Anzeige: Alle 2s
        self.timer2 = Timer(period=3000, mode=Timer.PERIODIC, callback=self._cb2) # Messung: Alle 3s
        self.timer3 = Timer(period=10*60*1000, mode=Timer.PERIODIC, callback=self._cb3) # Loggen: Alle 10min
    def stop(self):
        self.timer1.deinit()
        self.timer2.deinit()
        self.timer3.deinit()

    # These call backs are interrupt driven, hence complicated functions are not allowed
    # We use micropython.schedule to start the "real" worker
    # We war not allowed to allocate memory in the ISR See
    # https://docs.micropython.org/en/latest/reference/isr_rules.html#isr-rulese
    def _cb1(self, tim):
        micropython.schedule(self.display_ref, tim)
    def _cb2(self, tim):
        micropython.schedule(self.measure_ref, tim)
    def _cb3(self, tim):
        micropython.schedule(self.logdta_ref, tim)

def measure(args=None):
    global h1, t1, h2, t2
    led.blink()
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
    lcd.clear()
    lcd.move_to(0,0)
    out1 = f"{_t1:3.0f}{chr(grad)}C|{_h1:2.0f}%|{Taupunkt_1:4.1f}{chr(grad)}C"
    out2 = f"{_t2:3.0f}{chr(grad)}C|{_h2:2.0f}%|{Taupunkt_2:4.1f}{chr(grad)}C"
    lcd.putstr(out1)
    lcd.move_to(0,1)
    lcd.putstr(out2)
    DeltaTP = Taupunkt_1 - Taupunkt_2
    rel = False
    # Genaue Ausgabe auf Konsole
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
    else:
        Relais.on() # Relais ausschalten
        rgb_led.set(RGB_led.green)
    out  = f"S1: {t1:.2f}°C|{h1:.2f}%|{Taupunkt_1:.2f}°C | " + \
           f"S2: {t2:.2f}°C|{h2:.2f}%|{Taupunkt_2:.2f}°C | " + \
           f"DeltaTP {DeltaTP:.2f} | {not Relais.value()}"
    print(out)


def logdta(args=None,fname=LOGFILENAME, store=False):
    """
    Logge Werte in Buffer und einmal pro Tag in Datei fname
    """
    global logbuffer
    MAXLINES = 144
    dta = f"{pt()},{t1:.2f},{h1:.2f},{taupunkt(t1,h1):.2f},{t2:.2f},{h2:.2f},{taupunkt(t2,h2):.2f},{not Relais.value()}\n"
    print(dta)
    logbuffer.write(dta)
    if store or logbuffer.getvalue().count("\n") > MAXLINES:
        with open(LOGFILENAME,"a") as f:
            state = machine.disable_irq() # Sicher stellen, dass logbuffer unverändert bleibt
            logbuffer.flush()
            log = logbuffer.getvalue() # Lese gesamten Puffer
            logbuffer = io.StringIO() # Puffer löschen
            machine.enable_irq(state)
            print("Logbuffer gelöscht")
            f.write(log)

# Setup
# Logfile vorbereiten
try:
    with open(LOGFILENAME,"rt") as f:
        pass
except OSError:
    # Datei existiert nicht, erzeuge und schreibe Header
    with open(LOGFILENAME,"wt") as f:
        f.write("Date,t1,h1,tp1,t2,h2,tp2,fan\n")


dht1 = dht.DHT22(Pin(DHTPIN_1))   # Der Innensensor wird ab jetzt mit dht1 angesprochen
dht2 = dht.DHT22(Pin(DHTPIN_2))   # Der Außensensor wird ab jetzt mit dht2 angesprochen
Relais = Pin(RELAIPIN,Pin.OUT)
Relais.on() # Relais ausschalten
rgb_led = RGB_led()
led = Led()
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