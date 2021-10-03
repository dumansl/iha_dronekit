import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
import time
import math

def velocityPositionCallback(velocityPositionCallback):
    global vx
    global vy
    vx = velocityPositionCallback.vx
    vy = velocityPositionCallback.vy

class ardupilot_controller:
    def __init__(self):
        #service
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        #sub
        rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, velocityPositionCallback)
    def mode(self):
        if self.flightModeService(custom_mode='GUIDED'):
            return True
        else:
            return False 
    
    def land(self):
        if self.flightModeService(custom_mode='LAND'):
            return True
        else:
            return False

    def arm(self):
        if self.armService(True):
            return True
        else:
            return False
    def takeoff(self):
        if self.takeoffService(altitude = 3):
            return True
        else:
            return False

    def velocity_location(self, vx, vy):
        rospy.init_node('anadolu_kartali')
        guided_waypoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        while guided_waypoint_pub.get_num_connections() == 0:
            time.sleep(.1)
        g = PositionTarget()
        
        g.velocity.x = vx
        g.velocity.y = vy
       
        g.type_mask = int('110111000111', 2)
        g.coordinate_frame= 9

        global velocity

        guided_waypoint_pub.publish(g)



###################################################
if __name__ == '__main__':
    con = ardupilot_controller()
    con.mode()
    con.arm()
    con.takeoff()
    time.sleep(15)      

    con.velocity_location(5,0)
