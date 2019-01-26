from time import sleep
from ev3dev.auto import *


lm = LargeMotor('outA')
rm = LargeMotor('outB')
sm = MediumMotor('outC')

ir = InfraredSensor('in4')
ts = TouchSensor('in3')
cl1= ColorSensor('in1')
cl2 = ColorSensor('in2')
cl1.mode = 'COL-REFLECT'
cl2.mode = 'RGB-RAW'
ir.mode = 'IR-PROX'


power = 30
target = 30   #wartość zadana dla czujnika światła
kp = 0.8
kd = 1.0
ki = 0.2
minRead = 10
maxRead = 80

lastError = 0
error = 0
integral = 0

def steering(correction, power):    # Funkcja sterowania silnikami robota
	power_left = power_right = power
	s = (50 - abs(float(correction))) / 50

	if correction >= 0:
		power_right *= s
		if correction > 20:
			power_right = - power
	else:
		power_left *= s
		if correction < -50:
			power_left =  -power

	return (int(power_left), int(power_right))

print(ir.value())
lm.run_direct()
rm.run_direct()
while not ts.value():
	colRed = cl2.value(0)          #Odczyty czujnika dla kolorów kolejno czeronego, zielonego i niebieskiego
	colGreen = cl2.value(1)
	colBlue = cl2.value(2)
	lm.run_direct()
	rm.run_direct()
	clRead = cl1.value()
	error = target - (100 * (clRead - minRead)/(maxRead - minRead)) #różnica między wartością zadaną a wartością średnią
	derivative = error - lastError
	lastError = error
	integral = 0.5 * integral + error
	correction = kp * error + kd * derivative + ki * integral    #Regulator PID
	if(cl1.value() < 30 and colRed < 50 and colGreen < 60 and colBlue < 40):    #warunek do przejeżdżania przez skrzyżowania
		correction = 0;
	for (motor, pow) in zip((lm, rm), steering(correction, power)):   #Przekazanie korekcy sterowania
		motor.duty_cycle_sp = pow
	sleep(0.01)

lm.stop()
rm.stop()
