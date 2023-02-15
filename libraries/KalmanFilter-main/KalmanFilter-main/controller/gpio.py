import RPi.GPIO as gpio

class GPIO():
    def __init__(self,inputs=[],outputs=[]):
        gpio.setmode(gpio.BCM)
        self._inputs = inputs
        self._outputs = outputs
    
    def initialize(self):
        for input in self._inputs:
            gpio.setup(input,gpio.IN)
        for output in self._outputs:
            gpio.setup(output,gpio.OUT)

    def set_pin(self,pin,mode):
        if pin in self._outputs:
            gpio.output(pin,mode)
    
    def read_pin(self,pin):
        if pin in self._inputs:
            return gpio.input(pin)