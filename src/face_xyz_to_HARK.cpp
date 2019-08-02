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

//Subscriber (face_xyz_to_HARK.pyから角度情報を取得)

int face_xyz_array[90];

void publish_to_HARK(float x, float y, float z)
{
  //Publisher (HARKへ角度データを送信)
  // ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  // ros::Publisher HarkSource_pub = n.advertise<std_msgs::String>("HarkSource", 1000);
  ros::Publisher HarkSource_pub = n.advertise<hark_msgs::HarkSource>("HarkSource", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {

    hark_msgs::HarkSource HarkSourceMsg;

    HarkSourceMsg.count = 1;
    ros::Time oHarkTime = ros::Time::now();
    HarkSourceMsg.header.stamp = oHarkTime;
    //HarkSourceMsg.header.stamp.sec        = BaseTime.time.tv_sec;
    //HarkSourceMsg.header.stamp.nsec        = BaseTime.time.tv_usec * 1000;
    HarkSourceMsg.header.frame_id = "HarkRosFrameID";
    HarkSourceMsg.exist_src_num = 1;

    //header
    hark_msgs::HarkSourceVal HarkSourceValMsg;

    std::cout << "x:" << x << std::endl;
    std::cout << "y:" << y << std::endl;
    std::cout << "z:" << z << std::endl;


    HarkSourceValMsg.id    = 0;
  	HarkSourceValMsg.power = 38.5952;
  	HarkSourceValMsg.x     = x;
  	HarkSourceValMsg.y     = y;
  	HarkSourceValMsg.z     = z;
  	HarkSourceValMsg.azimuth   = 180.0 / M_PI * atan2(y, x);
  	HarkSourceValMsg.elevation = 180.0 / M_PI * atan2(z, sqrt(x * x + y * y));
  	HarkSourceMsg.src.push_back(HarkSourceValMsg);


    ROS_INFO("sending a message");
    HarkSource_pub.publish(HarkSourceMsg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}


// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// void callback(const hark_msgs::HarkSource::ConstPtr& msg)
void callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  ROS_INFO("Received message!");
  std::cout << *msg;
  ROS_INFO("============================");
  // ROS_INFO("I heard: [%s]", msg->data.c_str());

  int i = 0;
  int x, y, z;
  // print all the remaining numbers
  for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
  {
      face_xyz_array[i] = *it;
      i++;
  }

  x = face_xyz_array[2];
  y = face_xyz_array[0];
  z = face_xyz_array[1];

  publish_to_HARK(x, y, z); //HARKへx,y,zの座標を送信


}

int main(int argc, char **argv)
{
  //Subscriber (face_xyz_to_HARK.pyから角度情報を取得)
  ros::init(argc, argv, "face_xyz_to_HARK_cpp");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("face_recog_result", 1000, callback);
  ROS_INFO("Waiting for message ...");
  ros::spin();

  return 0;
}
