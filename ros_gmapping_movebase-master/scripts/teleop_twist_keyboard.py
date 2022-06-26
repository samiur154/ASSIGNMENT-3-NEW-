#!/usr/bin/env python
from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from sensor_msgs.msg import LaserScan
import os
import time

"""
    teleop twist keyboard node:
        it can implement two behaviours on robot:

        1: moving without obstacle avoidance:
            user can move the robot using keys it publishes the desired movements
            to cmd_vel topic

        2: moveing with obstacle avoidance:
            subscribes scan topic and uses it to detect obstacles. user 
            can move the robot using keys and it also avoids the robot
            from colliding the obstacles
"""

front_obs = False
right_obs = False
left_obs = False
mesg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to change robot behaviour
"""

# binding input keys for moving the robot
moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

# callback function for subscring laser scan topic 
def clbk_laser(msg):
    global front_obs 
    global right_obs 
    global left_obs
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
    if regions['front'] < 1:
        front_obs = True
    else:
        front_obs = False
    if regions['right'] < 1:
        right_obs = True
        # print("right obs detected")
    else:
        right_obs = False
    if regions['left'] < 1:
        left_obs = True
        # print("left obs detected")
    else:
        left_obs = False

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        global front_obs
        global right_obs
        global left_obs 
        twist = Twist()
        while not self.done:
            self.condition.acquire()

            if rospy.get_param('robot_state')=='2':
                # Copy state into twist message.
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.th * self.turn

            elif rospy.get_param('robot_state')=='3':
                # avoids the robot from moving forward if there is an obstacle
                if front_obs == True and self.x == 1:
                    twist.linear.x = 0
                # otherwise it can move forward
                else:
                    twist.linear.x = self.x * self.speed 
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0
                twist.angular.y = 0
                # avoids the robot from turning to right if there is an obstacle
                if right_obs == True and self.th == -1:
                    twist.angular.z = 0
                # avoids the robot from turning to left if there is an obstacle
                elif left_obs == True and self.th == 1:
                    twist.angular.z = 0
                # otherwise robot can turn in any direction
                else:
                    twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def teleop():
    settings = termios.tcgetattr(sys.stdin)
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        os.system('cls||clear')
        print("** TELEOP TWIST KEYBOARD NODE **\n")
        print(mesg)
        print(vels(speed,turn))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(mesg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    rospy.set_param('robot_state', '0')
                    os.system('cls||clear')
                    print("** TELEOP TWIST KEYBOARD NODE **\n")
                    print("choose robot behaviour in master node")
                    time.sleep(5)
                    os.system('cls||clear')
                    print("** TELEOP TWIST KEYBOARD NODE **\n")
                    print("waiting for master node response...\n")
                    break
 
            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



def main():
    rospy.init_node('teleop_twist_keyboard')
    rospy.set_param('robot_state', '0')
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    os.system('cls||clear')
    print("** TELEOP TWIST KEYBOARD NODE **\n")
    print("waiting for master node response...\n")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if rospy.get_param('robot_state')=='2' or rospy.get_param('robot_state')=='3':
            teleop()
        else:
            rate.sleep()
            continue
        rate.sleep()

if __name__ == '__main__':
    main()

