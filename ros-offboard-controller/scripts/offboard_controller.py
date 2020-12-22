import rospy
import csv
import os

from mavros_msgs.msg import *
from mavros_msgs.srv import *

from geographiclib.geodesic import Geodesic
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
from sensor_msgs.msg import NavSatFix


class FlightControlUnitModes:
    def __init__(self):
        pass

    def set_arm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def set_disarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def set_offboard(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set." % e)


class Controller:
    # initialization method
    def __init__(self):

        self.state = State()  # Drone state [Armed, Disarmed, ...]
        self.setpoint = GeoPoseStamped()  # Instantiate a set points message

        self.current_position = GeoPoint(0.0, 0.0, 0.0)

        self.acceptance_radius = 1.0

        self.geod = Geodesic.WGS84  # define the WGS84 ellipsoid # used in calculating distances between GPS points

        self.waypoint_coordinates = []
        self.waypoint_index = 0
        self.mission_complete = False

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl.

    def load_mission(self, file_name):
        self.mission_complete = False
        self.waypoint_index = 0

        basePath = os.path.dirname(os.path.abspath(__file__))

        with open(basePath + '/' + file_name, 'r') as read_obj:  # open file in read mode
            csv_reader = csv.reader(read_obj)  # pass the file object to reader() to get the reader object
            self.waypoint_coordinates = list(csv_reader)  # Pass reader object to list() to get a list of lists

    def global_position_callback(self, msg):
        self.current_position.latitude = msg.latitude
        self.current_position.longitude = msg.longitude

    def state_callback(self, msg):
        self.state = msg

    def init_global_position_callback(self):
        msg = rospy.wait_for_message('/mavros/global_position/global', NavSatFix)
        self.setpoint.pose.position.latitude = msg.latitude
        self.setpoint.pose.position.longitude = msg.longitude

    def take_image(self): # dummy function for taking images
        rospy.loginfo('Image captures')

    def update_setpoint(self):
        # Documentation for distance calculation
        # https: // geographiclib.sourceforge.io / html / python / code.html
        # https://geographiclib.sourceforge.io/html/python/interface.html#dict

        # Calculate distance to point
        geodesic_dictionary = self.geod.Inverse(self.setpoint.pose.position.latitude,
                                                self.setpoint.pose.position.longitude,
                                                self.current_position.latitude, self.current_position.longitude)
        distance = geodesic_dictionary['s12']

        if distance <= self.acceptance_radius:
            # take image then go to next waypoint

            self.take_image()

            self.waypoint_index += 1
            if self.waypoint_index < len(self.waypoint_coordinates):
                self.setpoint.pose.position.latitude = float(self.waypoint_coordinates[self.waypoint_index][0])
                self.setpoint.pose.position.longitude = float(self.waypoint_coordinates[self.waypoint_index][1])
            else:
                self.mission_complete = True


class OffBoardController:
    def __init__(self):
        rospy.init_node('offboard_node', anonymous=True)
        self.rate = rospy.Rate(20.0)

        self.modes = FlightControlUnitModes()  # flight mode object
        self.controller = Controller()  # controller object

        # Subscribes
        rospy.Subscriber('mavros/state', State, self.controller.state_callback)  # Subscribe to drone state
        rospy.Subscriber('/mavros/global_position/global', NavSatFix,
                         self.controller.global_position_callback)  # Subscribe to drones gps

        # Publisher
        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped,
                                                  queue_size=1)  # Publish global setpoint coordinates

        print('Offboard controller initialized')

    def load_mission(self, file_name):
        self.controller.load_mission(file_name)

    def arm(self):
        while not self.controller.state.armed:
            self.modes.set_arm()
            self.rate.sleep()

    def disarm(self):
        while self.controller.state.armed:
            self.modes.set_disarm()
            self.rate.sleep()

    def set_offboard_mode(self):
        # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
        k = 0
        while k < 10:
            self.setpoint_publisher.publish(self.controller.setpoint)
            self.rate.sleep()
            k += 1

        self.modes.set_offboard()

    def mission(self):
        print("Enter mission function")

        self.controller.init_global_position_callback()
        while (not rospy.is_shutdown()) and (not self.controller.mission_complete):
            self.controller.update_setpoint()
            self.setpoint_publisher.publish(self.controller.setpoint)
            self.rate.sleep()
