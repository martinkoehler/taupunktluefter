import dht
import math
from machine import Pin, I2C, WDT
from time import sleep_ms
from machine_i2c_lcd import I2cLcd
# Define constants
RELAIPIN = 6 # Anschluss des Lüfter-Relais

DHTPIN_1 = 2  # Datenleitung für den DHT-Sensor 1 (innen)
DHTPIN_2 = 10 # Datenleitung für den DHT-Sensor 2 (außen)

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

# Setup
def main():
    dht1 = dht.DHT22(Pin(DHTPIN_1))   # Der Innensensor wird ab jetzt mit dht1 angesprochen
    dht2 = dht.DHT22(Pin(DHTPIN_2))   # Der Außensensor wird ab jetzt mit dht2 angesprochen
    Relais = Pin(RELAIPIN,Pin.OUT)
    Relais.on() # Relais ausschalten
    # Initialisierung I2C
    i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=100000)
    # Initialisierung LCD über I2C
    lcd = I2cLcd(i2c, 0x27, 2, 16) # LCD: I2C-Addresse und Displaygröße setzen
    lcd.backlight_on()                      
    lcd.move_to(0,0)
    lcd.putstr("Teste Sensoren..")
    sleep_ms(2000)                    # Time for sensor setup
                                                                                                                                                                                                                                                # Watchdog timer auf 8 Sekunden stellen

    # Main loop
    firstrun = True
    while (1):
        fehler = False
        lcd.clear()
        try:
            dht1.measure()                           # Sensoren lesen
            h1 = dht1.humidity()+Korrektur_h_1       # Innenluftfeuchtigkeit auslesen und unter „h1“ speichern
            t1 = dht1.temperature()+ Korrektur_t_1   # Innentemperatur auslesen und unter „t1“ speichern
        except OSError as e:
            fehler = True
        if (fehler == True or h1 > 100 or h1 < 1 or t1 < -40 or t1 > 80 ):
            lcd.move_to(0,0)
            lcd.putstr("Fehler Sensor 1")
        elif (firstrun):
            lcd.move_to(0,0)
            lcd.putstr("Sensor 1 ok")
        try:
            dht2.measure()                           # Sensoren lesen
            h2 = dht2.humidity()+Korrektur_h_2       # Außenluftfeuchtigkeit auslesen und unter „h2“ speichern
            t2 = dht2.temperature()+ Korrektur_t_2   # Außentemperatur auslesen und unter „t2“ speichern
        except OSError as e:
            fehler = True
        if (fehler == True or h2 > 100 or h2 < 1 or t2 < -40 or t2 > 80 ):
            lcd.move_to(0,1)
            lcd.putstr("Fehler Sensor 2")
        elif (firstrun):
            lcd.move_to(0,1)
            lcd.putstr("Sensor 2 ok")
        firstrun = False
        sleep_ms(1000)  # Zeit um das Display zu lesen
        if (fehler == True):
            Relais.on() # "High": Lüfter "aus", da bei "low" Lüfter "an"
            lcd.clear()
            lcd.move_to(0,0)
            lcd.putstr("CPU Neustart...")
            #while (1):  # Endlosschleife um das Display zu lesen und die CPU durch den Watchdog neu zu starten
            #    pass
        #wdt.feed()  # Watchdog zurücksetzen

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
        t1 = round(t1) # Convert to integer
        h1 = round(h1) # Convert to integer
        t2 = round(t2) # Convert to integer
        h2 = round(h2) # Convert to integer
        # Genaue Ausgabe auf Konsole
        out  = "S1: {:4.2}°C|{:2.2}%|{:4.2f}°C".format(t1,h1,Taupunkt_1)
        out += "\n"
        out += "S2: {:4.2}°C|{:2.2}%|{:4.2f}°C".format(t2,h2,Taupunkt_2)
        print(out)
        lcd.clear()
        lcd.move_to(0,0)
        out1 = ("{:3}"+chr(grad) +"C|{:2}%|{:4.1f}"+chr(grad)+"C").format(t1,h1,Taupunkt_1)
        out2 = ("{:3}"+chr(grad)+"C|{:2}%|{:4.1f}"+chr(grad)+"C").format(t2,h2,Taupunkt_2)
        lcd.putstr(out1)
        lcd.move_to(0,1)
        lcd.putstr(out2)
        sleep_ms(6000) # Zeit um das Display zu lesen
        #wdt.feed() # Watchdog zurücksetzen

        lcd.clear()
        lcd.move_to(0,0)
        DeltaTP = Taupunkt_1 - Taupunkt_2

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
            lcd.putstr("L" + chr(ue) + "ftung AN")
            lcd.backlight_on()
        else:
            Relais.on() # Relais ausschalten
            lcd.putstr("L" + chr(ue) + "ftung AUS")
            #lcd.backlight_off()
        
        lcd.move_to(0,1)
        lcd.putstr(("Delta TP:{:4.1f}"+chr(grad)+"C").format(DeltaTP))
        sleep_ms(3000)     # Wartezeit zwischen zwei Messungen
        #wdt.feed()   # Watchdog zurücksetzen 

# Enable Watchdog
#wdt = WDT(timeout=8000)
main()