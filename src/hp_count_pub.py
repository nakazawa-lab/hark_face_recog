from harkpython import harkbasenode
import rospy
from std_msgs.msg import Int32

class HarkModalPub(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("initialized HarkModalPub")
        self.outputNames = ("OUTPUT",)
        self.outputTypes = ("prim_float",)
        self.c = 0

        rospy.init_node('hark_python_count', anonymous=True)
        self._count_pub = rospy.Publisher('hp_count', Int32, queue_size=10)

    def calculate(self):
        self.outputValues["OUTPUT"] = 1
        self._count_pub.publish(self.c)

        # print("=" * 14 + str(type(self.INPUT)) + "=" * 14)
        # print(self.INPUT)
        # print("frame no." + str(self.c))
        # print("")

        self.c = self.c + 1
