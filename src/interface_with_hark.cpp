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
//        void publish_to_hark(float x, float y, float z, int id);
        void publish_to_hark(int data_array[10][10], int num_speaker);
        ros::NodeHandle n;
        ros::Publisher HarkSource_pub = n.advertise<hark_msgs::HarkSource>("HarkSource", 10);
        int count = 0;
};

//void HarkInterface::callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
//{
//    int i = 0;
//    int x, y, z, id;
//    // print all the remaining numbers
//    for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
//    {
//      face_xyz_array[i] = *it;
//      i++;
//    }
//    // kinectの座標系とHARKの座標系を合わせる
//    x = face_xyz_array[2];
//    y = face_xyz_array[0];
//    z = face_xyz_array[1];
//    id = face_xyz_array[3];
//    HarkInterface::publish_to_hark(x, y, z, id);
////    ROS_INFO("x:%d, y:%d, z:%d, id:%d", x, y, z, id);
//}

void HarkInterface::callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    int i = 0;
    int num_speaker = 0;
    int human_data[10][10]; // 話者の位置とtalk_idを格納する２次元配列
    // subscriberしたメッセージを取り出して、face_xyz_arrayに格納する
    for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    {
      face_xyz_array[i] = *it;
      i++;
    }
    // 何人分のデータが送られてきたか計算
    num_speaker = i / 4;
    std::cout << "num_speaker:" << num_speaker << std::endl;
    for(int human_id=0; human_id<num_speaker; human_id++)
    {
      // kinectの座標系とHARKの座標系を合わせる
      // 複数人のデータを{0:x, 1:y, 2:z, 3:talk_id}の順番で格納
      human_data[human_id][0] = face_xyz_array[4 * human_id + 2];
      human_data[human_id][1] = face_xyz_array[4 * human_id ];
      human_data[human_id][2] = face_xyz_array[4 * human_id + 1];
      human_data[human_id][3] = face_xyz_array[4 * human_id + 3];
    }
    HarkInterface::publish_to_hark(human_data, num_speaker);
}

void HarkInterface::publish_to_hark(int data_array[10][10], int num_speaker)
{
    int x, y, z, talk_id;
    float theta;
    hark_msgs::HarkSource HarkSourceMsg;
    HarkSourceMsg.count = count;
    ros::Time oHarkTime = ros::Time::now();
    HarkSourceMsg.header.stamp = oHarkTime;
    HarkSourceMsg.header.frame_id = "HarkRosFrameID";
    HarkSourceMsg.exist_src_num = num_speaker;
    // 話者ごとに位置と方向、talk_idをHarkSourceValMsgに格納 → HarkSourceMsgにまとめて格納し、HARKへ送信
    for(int human_id=0; human_id<num_speaker; human_id++)
    {
        x = data_array[human_id][4 * human_id];
        y = data_array[human_id][4 * human_id + 1];
        z = data_array[human_id][4 * human_id + 2];
        talk_id = data_array[human_id][4 * human_id + 3];
        theta = 180.0 / M_PI * atan2(y, x);
        // 話者の位置と方向、talk_idをHarkSourceValMsgに格納
        hark_msgs::HarkSourceVal HarkSourceValMsg;
        HarkSourceValMsg.x     = x;
        HarkSourceValMsg.y     = y;
        HarkSourceValMsg.z     = z;
        HarkSourceValMsg.id    = talk_id;
        HarkSourceValMsg.azimuth   = theta;
        HarkSourceValMsg.elevation = 180.0 / M_PI * atan2(z, sqrt(x * x + y * y));
        // 口が動いていない場合
        if(x==0 && y==0 && z==0){
          HarkSourceValMsg.power = 0;
        }
        // 口が動いている場合
        else{
          HarkSourceValMsg.power = 1.0;
        }
        ROS_INFO("id:%d, theta: %ddeg, power: %d, y:%d, z:%d",talk_id, (int)theta, (int)HarkSourceValMsg.power, (int)y,(int)z);
        // データをHarkSourceMsgに格納
        HarkSourceMsg.src.push_back(HarkSourceValMsg);
    }
    // HARKへデータを送信
    HarkSource_pub.publish(HarkSourceMsg);
    ++count;
}

//void HarkInterface::publish_to_hark(float x, float y, float z, int id)
//{
//    hark_msgs::HarkSource HarkSourceMsg;
//    HarkSourceMsg.count = count;
//    ros::Time oHarkTime = ros::Time::now();
//    HarkSourceMsg.header.stamp = oHarkTime;
//    HarkSourceMsg.header.frame_id = "HarkRosFrameID";
//    HarkSourceMsg.exist_src_num = 1;
//
//    float theta = 180.0 / M_PI * atan2(y, x);
//    //header
//    hark_msgs::HarkSourceVal HarkSourceValMsg;
//    HarkSourceValMsg.id    = id;
//    HarkSourceValMsg.x     = x;
//    HarkSourceValMsg.y     = y;
//    HarkSourceValMsg.z     = z;
//    HarkSourceValMsg.azimuth   = theta;
//    HarkSourceValMsg.elevation = 180.0 / M_PI * atan2(z, sqrt(x * x + y * y));
//
//    // 口が動いていない場合
//    if(x==0 && y==0 && z==0){
//      HarkSourceValMsg.power = 0;
//    }
//    // 口が動いている場合
//    else{
//      HarkSourceValMsg.power = 1.0;
//    }
//    ROS_INFO("id:%d, theta: %ddeg, power: %d, y:%d, z:%d",id, (int)theta, (int)HarkSourceValMsg.power, (int)y,(int)z);
//
//    HarkSourceMsg.src.push_back(HarkSourceValMsg);
//    HarkSource_pub.publish(HarkSourceMsg);
//    ++count;
//}

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
