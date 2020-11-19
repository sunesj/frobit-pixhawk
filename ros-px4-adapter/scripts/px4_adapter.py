import rospy
from std_msgs.msg import Float64
from mavros_msgs.msg import RCOut

input_max = 2000.0
input_min = 1000.0
input_dif = input_max - input_min

output_max = 1.78
output_min = -1.78
output_dif = output_max - output_min

class PX4Adapter():
    def __init__(self):
        rospy.init_node('px4_adapter_node', anonymous=True)

        self.pub_left = rospy.Publisher("/frobit/left_setpoint", Float64, queue_size=1)
        self.pub_right = rospy.Publisher("/frobit/right_setpoint", Float64, queue_size=1)
        self.sub = rospy.Subscriber("/mavros/rc/out", RCOut, self.subscriber_callback)


    def subscriber_callback(self, msg):
        left_msg = Float64()
        right_msg = Float64()

        left_msg.data = output_min + (output_dif * (msg.channels[0] - input_min) / input_dif)
        right_msg.data = output_min + (output_dif * (msg.channels[1] - input_min) / input_dif)

        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)


    def spin(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            print("px4_adapter_node alive and well")
            r.sleep()
