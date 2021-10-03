import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
import time
import math

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude

def get_distance_metres(latitude,longitude, lat, lon):
    dlat = latitude - lat
    dlong = longitude - lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

class ardupilot_controller:
    def __init__(self):
        #service
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        #sub
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    def mode(self):
        if self.flightModeService(custom_mode='GUIDED'):
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

    def global_location(self, alt, lat, lon):
        rospy.init_node('anadolu_kartali')
        guided_waypoint_pub = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
        while guided_waypoint_pub.get_num_connections() == 0:
            time.sleep(.1)
        g = GlobalPositionTarget()
        g.altitude = alt
        g.latitude = lat
        g.longitude = lon
        g.type_mask=4088
        g.coordinate_frame=6

        global latitude
        global longitude
        
        targetDistance = get_distance_metres(latitude,longitude, lat, lon)
        guided_waypoint_pub.publish(g)

        while self.flightModeService(custom_mode='GUIDED'): 
            remainingDistance=get_distance_metres(latitude,longitude, lat, lon)
            print("Distance to target: ", remainingDistance)
            if remainingDistance<=targetDistance*0.025:
                print("Reached target")
                break;
            time.sleep(2)


###################################################
if __name__ == '__main__':
    con = ardupilot_controller()
    con.mode()
    con.arm()
    con.takeoff()
    time.sleep(15)      

    con.global_location(5, -35.36245116, 149.16517902)
    con.global_location(5, -35.36215729, 149.16515781)
    con.global_location(5, -35.36217458, 149.16500236)
    con.global_location(5, -35.36327865, 149.16513780)


