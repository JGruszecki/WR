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


power = 30   #dla power 40 jedzie gladko
target = 30
kp = 0.8
kd = 1.0
ki = 0.2
minRead = 10
maxRead = 80

lastError = 0
error = 0
integral = 0


redDetected = 0
greenDetected = 0;
transporting = 0
objLifted = 0
objDetected = 0;
lastColor = 42

def steering(correction, power):
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
	colRed = cl2.value(0)
	colGreen = cl2.value(1)
	colBlue = cl2.value(2)
	lm.run_direct()
	rm.run_direct()
	clRead = cl1.value()
	error = target - (100 * (clRead - minRead)/(maxRead - minRead))
	derivative = error - lastError
	lastError = error
	integral = 0.5 * integral + error
	correction = kp * error + kd * derivative + ki*integral
	if(cl1.value() < 30 and colRed < 50 and colGreen < 60 and colBlue < 40):    #warunek do przejeżdżania przez skrzyżowania
		correction = 0;
	for (motor, pow) in zip((lm, rm), steering(correction, power)):
		motor.duty_cycle_sp = pow




	sleep(0.01)
if(ir.value() > 20 and objLifted == 1):
	sm.run_to_rel_pos(position_sp = 200, speed_sp = 800, stop_action = "hold")
	podniesiony = 0;
lm.stop()
rm.stop()
