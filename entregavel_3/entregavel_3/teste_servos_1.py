import RPi.GPIO as GPIO
import time

# ==== CONFIGURACAO ====
pin_braco = 12
pin_garra = 13
freq = 50

# Mesmos ganhos do código original
def pos_braco(pwm, valor):
    valor = float(valor)
    commandBraco = 5 + 2.67*valor + (0.444*(valor**2))
    print("Duty Braco:", commandBraco)
    pwm.ChangeDutyCycle(commandBraco)

def pos_garra(pwm, valor):
    commandGarra = (6 * valor) + 12
    print("Duty Garra:", commandGarra)
    pwm.ChangeDutyCycle(commandGarra)

# ==== INICIO ====

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(pin_braco, GPIO.OUT)
GPIO.setup(pin_garra, GPIO.OUT)

pwm_braco = GPIO.PWM(pin_braco, freq)
pwm_garra = GPIO.PWM(pin_garra, freq)

pwm_braco.start(0)
pwm_garra.start(0)

try:
    print("\n=== Teste do Servo Braco (varrendo comandos) ===")
    for v in [ -1.0, -0.5, 0.0, 0.5, 1.0 ]:
        print("\nBraco -> comando =", v)
        pos_braco(pwm_braco, v)
        time.sleep(1)
        pwm_braco.ChangeDutyCycle(0)
        time.sleep(0.5)

    print("\n=== Teste do Servo Garra (varrendo comandos) ===")
    for v in [ -1.0, -0.5, 0.0, 0.5, 1.0 ]:
        print("\nGarra -> comando =", v)
        pos_garra(pwm_garra, v)
        time.sleep(1)
        pwm_garra.ChangeDutyCycle(0)
        time.sleep(0.5)

except KeyboardInterrupt:
    pass

print("Encerrando...")
pwm_braco.stop()
pwm_garra.stop()
GPIO.cleanup()
