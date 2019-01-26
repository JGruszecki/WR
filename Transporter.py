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


redDetected = 0       #Flagi pomocnicze do skrecania na kolorowe pola
redDetected2 = 0
greenDetected = 0
transporting = 0
objLifted = 0
objDetected = 0

lastColor = 42

def steering(correction, power):      #Funkcja sterująca
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

	"""Sekwencja skrecania na zielonym do odebrania obiektu"""
	if(cl2.value() < 100 and cl2.value(1) >= 120 and cl2.value(2) < 100 and greenDetected == 0 and transporting == 0):
		sleep(1.2)
		lm.run_to_rel_pos(position_sp = 270, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = -270, speed_sp = 100, stop_action = "hold")
		sleep(5)
		lm.run_to_rel_pos(position_sp = 130, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = 130, speed_sp = 100, stop_action = "hold")
		sleep(4)
		greenDetected = 1
		print("SKRET_zielony")

	"""Sekwencja podnoszenia obiektu na zielonym polu"""
	if(cl2.value() < 100 and cl2.value(1) >= 120 and cl2.value(2) < 100 and greenDetected == 1 and transporting == 0 and transporting == 0 and objLifted == 0):
		lm.stop()
		rm.stop()
		sleep(1)
		while(ir.value() > 18):
			lm.run_forever(speed_sp = 100)
			rm.run_forever(speed_sp = 100)

		"""Podnoszenie obiektu"""
		lm.stop()
		rm.stop()
		sleep(1)
		sm.run_to_rel_pos(position_sp = -200, speed_sp = 800, stop_action = "hold")
		objLifted = 1
		objDetected = 1
		sleep(2)
		lm.run_to_rel_pos(position_sp = 560, speed_sp = 200, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = -560, speed_sp = 200, stop_action = "hold")
		sleep(5)
		lm.run_to_rel_pos(position_sp = 100, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = 100, speed_sp = 100, stop_action = "hold")
		greenDetected = 0
		transporting = 1


	"""Detekcja czerwonego i sekwencja skretu"""
	if(cl2.value(0) >= 200 and cl2.value(1) < 100 and cl2.value(2) < 100 and transporting == 1 and redDetected == 0):
		sleep(1)
		lm.run_to_rel_pos(position_sp = 270, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = -270, speed_sp = 100, stop_action = "hold")
		sleep(4)
		lm.run_to_rel_pos(position_sp = 100, speed_sp = 200, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = 100, speed_sp = 200, stop_action = "hold")
		sleep(2)
		redDetected = 1

	"""Sekwencja odkladania obiektu i wycofania sie na linie"""
	if(cl2.value(0) >= 200 and cl2.value(1) < 100 and cl2.value(2) < 100 and redDetected == 1):
		sleep(1)
		lm.stop()
		rm.stop()
		sm.run_to_rel_pos(position_sp = 200, speed_sp = 800, stop_action = "hold")
		sleep(1)
		lm.run_to_rel_pos(position_sp = -150, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = -150, speed_sp = 100, stop_action = "hold")
		sleep(3)
		lm.run_to_rel_pos(position_sp = -500, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = 500, speed_sp = 100, stop_action = "hold")
		sleep(3)
		transporting = 0
		redDetected = 0
		redDetected2 = 1

	"""Detekcja czerwonego i sekwencja skretu"""
	if(cl2.value(0) >= 200 and cl2.value(1) < 100 and cl2.value(2) < 100 and redDetected2 == 1):
		sleep(2)
		lm.run_to_rel_pos(position_sp = 280, speed_sp = 100, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = -280, speed_sp = 100, stop_action = "hold")
		sleep(4)
		lm.run_to_rel_pos(position_sp = 100, speed_sp = 200, stop_action = "hold")
		rm.run_to_rel_pos(position_sp = 100, speed_sp = 200, stop_action = "hold")
		sleep(2)
		redDetected2 = 0

	lm.run_direct()
	rm.run_direct()
	clRead = cl1.value()
	error = target - (100 * (clRead - minRead)/(maxRead - minRead))
	derivative = error - lastError
	lastError = error
	integral = 0.5 * integral + error
	correction = kp * error + kd * derivative + ki*integral     #Sterowanie PID
	if(cl1.value() < 40 and cl2.value(0) < 60 and cl2.value(1) < 70 and cl2.value(2) < 50):    #warunek do przejeżdżania przez skrzyżowania
		correction = 0;
	for (motor, pow) in zip((lm, rm), steering(correction, power)):
		motor.duty_cycle_sp = pow

	print("STOP")
	sleep(0.01)



lm.stop()
rm.stop()
