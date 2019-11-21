#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hark_msgs/HarkSource.h"
#include <typeinfo>



void publish_to_python_file()
{

    //Publisher (pythonファイルへstartという文字列を送信)
    ros::NodeHandle n;
    //Publisherの定義
    ros::Publisher source_pub = n.advertise<std_msgs::String>("start_sign", 1000);

     ros::Rate loop_rate(100);
    int count = 0;

    while(ros::ok())
    {
        // 文字列の定義
        std_msgs::String msg;
        std::stringstream ss;
        ss << "start";
        msg.data = ss.str();
        std::cout << "sending start message" << std::endl;
        std::cout << std::endl;
        source_pub.publish(msg);
    }
}

void callback(const hark_msgs::HarkSource::ConstPtr& msg)
{
    ROS_INFO("Received message!");
    // std::cout << *msg;
    std::cout << "exist_src_num:" << msg->exist_src_num << std::endl;
    for(int i=0; i<msg->exist_src_num; i++){
    std::cout << "id:" << msg->src[i].id << std::endl;
    std::cout << "power:" <<msg->src[i].power << std::endl;
    std::cout << "x:" <<msg->src[i].x << std::endl;
    std::cout << "y:" << msg->src[i].y << std::endl;
    std::cout << "z:" << msg->src[i].z << std::endl;
    }

    if(msg->exist_src_num > 0){
        publish_to_python_file();
    }
}

int main(int argc, char **argv)
{
    // ノードの初期化
    ros::init(argc, argv, "ros_msgs_listener");

    ros::NodeHandle n;
    // Subscriberの定義
    ros::Subscriber source_sub = n.subscribe("HarkSource_sign", 1000, callback);

    ROS_INFO("Waiting for message ...");
    ros::spin();
    return 0;
}