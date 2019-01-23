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

	"""Sekwencja skrecania na czerwonym do odebrania obiektu"""
	if(colRed >= 260 and colGreen < 70 and colBlue < 40 and redDetected == 0 and transporting == 0):
		sleep(0.5)
		lm.run_to_rel_pos(position_sp = 270, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = -270, speed_sp = 100, stop_action = "hold")
		sleep(0.5)
		lm.run_to_rel_pos(position_sp = 200, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = 200, speed_sp = 100, stop_action = "hold")
		sleep(0.5)
		redDetected = 1

	"""Sekwencja podnoszenia obiektu na czerwonym polu"""
	if(colRed >= 260 and colGreen < 70 and colBlue < 40 and redDetected == 1):
		lm.stop()
		rm.stop()
		sleep(1)
		lm.run_forever(speed_sp = 100)
		rm.run_forever(speed_sp = -100)

		"""Podnoszenie obiektu"""
		if(ir.value() < 18 and objLifted == 0 and ir.value() > 0 and redDetected == 1):
			lm.stop()
			rm.stop()
			sleep(2)
			sm.run_to_rel_pos(position_sp = -200, speed_sp = 800, stop_action = "hold")
			objLifted = 1
			objDetected = 1
			sleep(2)
			lm.run_to_rel_pos(position_sp = 360, speed_sp = 200, stop_action = "hold")
			rm.run_to_rel_pos(position_sp = -360, speed_sp = 200, stop_action = "hold")
			sleep(1)
			lm.run_to_rel_pos(position_sp = 360, speed_sp = 200, stop_action = "hold")
			rm.run_to_rel_pos(position_sp = 360, speed_sp = 200, stop_action = "hold")
			redDetected = 0
			"""
			while(colRed > 60 or colGren > 60 or colBlue > 70):
				lm.run_forever(speed_sp = 100)
				rm.run_forever(speed_sp = -100)
				lm.stop()
				rm.stop()
				"""

	"""Detekcja zielonego i sekwencja skretu"""
	if(colRed < 60 and colGren >= 150 and colBlue < 70 and transporting == 1):
		sleep(0.2)
		lm.run_to_rel_pos(position_sp = -245, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = 245, speed_sp = 100, stop_action = "hold")
		sleep(2)
		lm.run_to_rel_pos(position_sp = 200, speed_sp = 200, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = 200, speed_sp = 200, stop_action = "hold")
		sleep(2)
		greenDetected = 1

	"""Sekwencja odkladania obiektu i wycofania sie na linie"""
	if(colRed < 60 and colGren >= 150 and colBlue < 70 and greenDetected == 1):
		sleep(1)
		lm.stop()
		rm.stop()
		sm.run_to_rel_pos(position_sp = 200, speed_sp = 800, stop_action = "hold")
		sleep(1)
		lm.run_to_rel_pos(position_sp = -360, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = -560, speed_sp = 100, stop_action = "hold")
		sleep(1)
		lm.run_to_rel_pos(position_sp = -360, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = 360, speed_sp = 100, stop_action = "hold")
		greenDetected = 0


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
