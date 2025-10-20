docker run -it --rm \
  --device /dev/gpiomem \
  --group-add gpio \
  -v /sys:/sys \
  -v /proc:/proc \
  ros:humble
  
  
  
from gpiozero import LED
from time import sleep

led = LED(23)

try:
    while True:
        led.on()
        sleep(1)  # LED ON for 1 second
        led.off()
        sleep(1)  # LED OFF for 1 second
except KeyboardInterrupt:
    led.off()
    print("LED stopped")