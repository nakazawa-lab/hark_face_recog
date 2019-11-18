#include "ros/ros.h"
#include "hark_msgs/HarkSource.h"
#include "hark_msgs/HarkSourceVal.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>
#include <math.h>
#include <vector>
#include <typeinfo>

class HarkInterface
{
    private:
        int face_xyz_array[90];
    public:
        void callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void publish_to_hark(float x, float y, float z, int id);
        ros::NodeHandle n;
        ros::Publisher HarkSource_pub = n.advertise<hark_msgs::HarkSource>("HarkSource", 10);
        int count = 0;
};

void HarkInterface::callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    int i = 0;
    int x, y, z, id;
    // print all the remaining numbers
    for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    {
      face_xyz_array[i] = *it;
      i++;
    }
    // kinectの座標系とHARKの座標系を合わせる
    x = face_xyz_array[2];
    y = face_xyz_array[0];
    z = face_xyz_array[1];
    id = face_xyz_array[3];
    HarkInterface::publish_to_hark(x, y, z, id);
//    ROS_INFO("x:%d, y:%d, z:%d, id:%d", x, y, z, id);
}

void HarkInterface::publish_to_hark(float x, float y, float z, int id)
{
    hark_msgs::HarkSource HarkSourceMsg;
    HarkSourceMsg.count = count;
    ros::Time oHarkTime = ros::Time::now();
    HarkSourceMsg.header.stamp = oHarkTime;
    HarkSourceMsg.header.frame_id = "HarkRosFrameID";
    HarkSourceMsg.exist_src_num = 1;

    float theta = 180.0 / M_PI * atan2(y, x);
    //header
    hark_msgs::HarkSourceVal HarkSourceValMsg;
    HarkSourceValMsg.id    = id;
    HarkSourceValMsg.x     = x;
    HarkSourceValMsg.y     = y;
    HarkSourceValMsg.z     = z;
    HarkSourceValMsg.azimuth   = theta;
    HarkSourceValMsg.elevation = 180.0 / M_PI * atan2(z, sqrt(x * x + y * y));

    // 口が動いていない場合
    if(x==0 && y==0 && z==0){
      HarkSourceValMsg.power = 0;
    }
    // 口が動いている場合
    else{
      HarkSourceValMsg.power = 40.5952;
    }
    ROS_INFO("id:%d, theta: %ddeg, power: %d, y:%d, z:%d",id, (int)theta, (int)HarkSourceValMsg.power, (int)y,(int)z);

    HarkSourceMsg.src.push_back(HarkSourceValMsg);
    HarkSource_pub.publish(HarkSourceMsg);
    ++count;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface_with_hark");
    ros::NodeHandle n;
    HarkInterface hi;
    ros::Subscriber sub = n.subscribe("face_recog_result", 10, &HarkInterface::callback, &hi);
    ROS_INFO("Waiting for message ...");
    ros::spin();
    return 0;
}
