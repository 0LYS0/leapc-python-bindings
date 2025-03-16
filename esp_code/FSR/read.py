from machine import Pin, ADC
from time import sleep

p1 = Pin(1, Pin.IN)
p2 = Pin(2, Pin.IN)
p3 = Pin(3, Pin.IN)
p4 = Pin(4, Pin.IN)
p5 = Pin(5, Pin.IN)


adc1 = ADC(p1)
adc2 = ADC(p2)
adc3 = ADC(p3)
adc4 = ADC(p4)
adc5 = ADC(p5)

adc_list = [adc1, adc2, adc3, adc4, adc5]

def read_FSR_values():
    value_text = ""
    for adc in adc_list:
        value_text += str(int(adc.read_u16()/65535*255)) + ","
    
    return value_text
