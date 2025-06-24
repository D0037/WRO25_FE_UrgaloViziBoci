#!/usr/bin/python3
import utils.adc as adc

adc.init()
print(adc.read_bat())