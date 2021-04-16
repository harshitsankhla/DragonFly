import time
import rospy
import mavros
import threading

from mavros import param
from mavros import command
from mavros import utils

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, Altitude
from mavros_msgs.srv import SetMode

from geometry_msgs.msg import PoseStamped


class CreateThread:
    def __init__(self, func_name, interval=1):
        self.interval = interval
        self.function = func_name
        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True
        thread.start()

    def run(self):
        while True:
            self.function()
            time.sleep(self.interval)


class Vehicle:
    def __init__(self, config):
        self.alt_topic = config["alt_topic"]
        self.gps_topic = config["gps_topic"]
        self.state_topic = config["state_topic"]

        self.armed = False
        self.connected = False
        self.mode = ''
        self.takeoff = False
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0

        rospy.init_node("mavros_vehicle", anonymous=True)
        mavros.set_namespace()
        if mavros.utils.wait_fcu_connection(timeout=10):
            print("Connected to Flight Controller !")
            self.connected = True

        CreateThread(self.heartbeat)

        print("Listening to Altitude on %s" % self.alt_topic)
        print("Listening to GPS on - %s" % self.gps_topic)
        print("Listening to State on - %s" % self.state_topic)

        param.param_set("NAV_RCL_ACT", 0)
        param.param_set("NAV_DLL_ACT", 0)

    def arm(self):
        if self.armed:
            print ("Vehicle Already Armed !")
        else:
            print("Arming Now...")
            try:
                ret = command.arming(True)
                self.armed = True
                if not ret.success:
                    utils.fault("Request failed. Check mavros logs")
            except rospy.ServiceException as ex:
                utils.fault(ex)

        print("Arming Command result: %s" % (self.armed == True))

    def disarm(self):
        if not self.armed:
            print("Vehicle Already Disarmed !")
        else:
            print("Disarming Now...")
            try:
                ret = command.arming(False)
                self.armed = False
                if not ret.success:
                    utils.fault("Request failed. Check mavros logs")
            except rospy.ServiceException as ex:
                utils.fault(ex)

        print("Disarming Command result: %s" % (self.armed == False))

    def takeoff(self):
        if self.takeoff:
            print("Vehicle has already Taken Off")
        else:
            try:
                alt = param.param_get("MIS_TAKEOFF_ALT")
                print("Taking off at Current Latitude = %3f Longitude =  %3f" % (self.latitude, self.longitude))
                ret = command.takeoff(min_pitch=0, yaw=0, latitude=self.latitude, longitude=self.longitude,
                                      altitude=alt)
                while self.altitude <= (alt - 0.1):
                    continue
                self.takeoff = True
                if not ret.success:
                    utils.fault("Request failed. Check mavros logs")
            except rospy.ServiceException as ex:
                utils.fault(ex)

        print("Takeoff Command result: %s" % (self.takeoff == True))

    def land(self):
        if not self.takeoff:
            print("Vehicle has already Landed")
        else:
            try:
                print("Landing at current location !")
                ret = command.land(min_pitch=0, yaw=0, latitude=self.latitude, longitude=self.longitude, altitude=0)
                while self.altitude <= 0.2:
                    continue
                self.takeoff = False
                if not ret.success:
                    utils.fault("Request failed. Check mavros logs")
            except rospy.ServiceException as ex:
                utils.fault(ex)

        print("Land Command result: %s" % (self.takeoff == False))

    def offboard(self):
        if self.armed:
            rate = rospy.Rate(20.0)  # MUST be more then 2Hz
            set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
            local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)

            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = self.altitude

            last_request = rospy.get_rostime()
            while not rospy.is_shutdown():
                now = rospy.get_rostime()
                if self.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5)):
                    set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                    rospy.loginfo("Entering OFFBOARD MODE NOW ! Use local waypoints for flying")
                    last_request = now

                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = pose.pose.position.x + 0.05
                local_pos_pub.publish(pose)
                rate.sleep()
        else:
            rospy.loginfo("Vehicle is not armed, please arm first then proceed !")

    def heartbeat(self):
        try:
            state_msg = rospy.wait_for_message(self.state_topic, State, timeout=10)
            gps_msg = rospy.wait_for_message(self.gps_topic, NavSatFix, timeout=10)
            alt_msg = rospy.wait_for_message(self.alt_topic, Altitude, timeout=10)

            self.armed = state_msg.armed
            self.connected = state_msg.connected
            self.mode = state_msg.mode
            self.latitude = gps_msg.latitude
            self.longitude = gps_msg.longitude
            self.altitude = alt_msg.relative

        except Exception as e:
            print("Failed to Receive a Heartbeat from the Vehicle !")
