import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Mesmos pinos que você usa
pins = {
    "orelha_esq": 21,
    "orelha_dir": 20,
    "braco_esq": 16,
    "braco_dir": 12
}

servos = {}

# Inicializa cada PWM
for nome, pin in pins.items():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)  # 50 Hz = servos padrão
    pwm.start(0)
    servos[nome] = pwm

def mover_servo(pwm, angulo):
    """Converte ângulo para duty cycle e move o servo."""
    duty = 2 + (angulo / 18)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)


try:
    while True:
        print("Movendo servos para 0°...")
        for s in servos.values():
            mover_servo(s, 0)
        time.sleep(1)

        print("Movendo servos para 90°...")
        for s in servos.values():
            mover_servo(s, 90)
        time.sleep(1)

        print("Movendo servos para 180°...")
        for s in servos.values():
            mover_servo(s, 180)
        time.sleep(1)

except KeyboardInterrupt:
    print("Encerrando teste...")

finally:
    for s in servos.values():
        s.stop()
    GPIO.cleanup()