import ADS1x15
import config
import time
from utils.led import LED
import sys

def get_bat():
    """
    Reads the battery voltage using the ADS1115 ADC.
    The battery is connected to the ADC through a voltage divider.
    """
    ADS = ADS1x15.ADS1115(1)  # or ADS1x15.ADS1015() for 10-bit resolution
    ADS.setMode(ADS.MODE_SINGLE)
    ADS.setDataRate(ADS1x15.ADS1115.DR_ADS111X_8)
    ADS.setGain(ADS.PGA_4_096V) # Max voltage is 12.6V / (1321 / 328) = ~3.12V
    f = ADS.toVoltage()
    voltage = ADS.readADC(0) * f # Read from channel 0

    # Apply the conversion ratio for the voltage divider
    return voltage * config.BAT_V_CONVERSION

# This is used when the script is run by systemd, so an LED can be lit
def warn_bat():
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)

    warn = LED(config.LED_WARN)

    while True:
        if get_bat() < 10:
            warn.on()
        elif get_bat() > 10.1:
            warn.off()
        
        time.sleep(1)
    
if __name__ == "__main__":
    print(get_bat())

    if len(sys.argv) > 1 and sys.argv[1] == "warn":
        warn_bat()