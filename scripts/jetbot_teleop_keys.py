#!/usr/bin/env python

import rospy

from Adafruit_MotorHAT import Adafruit_MotorHAT
from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Jetbot Keyboard Control!
---------------------------
Teclas de movimiento:
        i    
   j    k    l

q/z : incrementa/decrece velocidad por un 10%

space key, k : stop forzoso
anything else : stop 

CTRL-C para salir
"""

moveBindings = {
        'i':(1,1),
        'j':(1,-1),
        'k':(-1,-1),
        'l':(-1,1)
          }

speedBindings={
        'q':(0.1, 0),
        'z':(0, 0.1)
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.3

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)

# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)


def vels(speed):
    return "Velocidad:\tactual %s" % (speed)

if __name__=="__main__":

    # setup motor controller
    motor_driver = Adafruit_MotorHAT(i2c_bus=1)

    motor_left_ID = 1
    motor_right_ID = 2

    motor_left = motor_driver.getMotor(motor_left_ID)
    motor_right = motor_driver.getMotor(motor_right_ID)
    
    # stop the motors as precaution
    all_stop()

    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('jetbot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    leftM = 0
    rightM = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    control_speed = 0

    try:
        print(msg)
        print(vels(speed))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                leftM = moveBindings[key][0]
                rightM = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0] - speedBindings[key][1]
		if (speed >= 1.0):
                    speed = 1.0
		elif (speed <= 0.1):
		    speed = 0.1
		   
                print(vels(speed))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                leftM = 0
                rightM = 0
                control_speed = 0
            else:
                count = count + 1
                if count > 4:
                    leftM = 0
                    rightM = 0
		    all_stop()
                if (key == '\x03'):
                    break

	    target_speed = speed

            leftSpeed = target_speed * leftM
	    rightSpeed = target_speed * rightM 
		
	    if(leftSpeed != 0 or rightSpeed != 0 ):
		set_speed(motor_left_ID,  1.0*leftSpeed)
		set_speed(motor_right_ID,  1.0*rightSpeed) 

            twist = Twist()
	    twist.linear.x = target_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	    pub.publish(twist)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except Exception as e:
	all_stop()
        print(e)

    finally:
	all_stop()
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

