#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include"wav_file_reader.h"


int main(void) {

    sakado::WavFileReader wfr("/home/nvidia/catkin_ws/src/hark_face_recog/src/userdata/records/movie_dataset/dataset2/channel0.wav");
    unsigned char buf[44100];

    wfr.Read(buf, 44100);//最初の44100サンプルを読み込み
    wfr.Read(buf, 44100);//次の44100サンプルを読み込み

    return 0;
}









//int main(int argc, char **argv)
//{
//
//    // wavファイル読み込み
//    sakado::WavFileReader wfr("/home/nvidia/catkin_ws/src/hark_face_recog/src/userdata/records/movie_dataset/dataset2/multichannel_audio.wav");
//    unsigned char buf[44100];
//
//    wfr.Read(buf, 44100);//最初の44100サンプルを読み込み
//    wfr.Read(buf, 44100);//次の44100サンプルを読み込み
//
//
//    ros::init(argc, argv, "send_wav_to_ROS");
//    ros::NodeHandle n;
//    ros::Publisher wave_pub = n.advertise<std_msgs::String>("HarkWave", 1000);
//    ros::Rate loop_rate(10);
//    int count = 0;
//
//    while (ros::ok())
//    {
//    std_msgs::String msg;
//    std::stringstream ss;
//    ss << "hello world " << count;
//    msg.data = ss.str();
//    ROS_INFO("%s", msg.data.c_str());
//    wave_pub.publish(msg);
//    ros::spinOnce();
//    loop_rate.sleep();
//    ++count;
//    }
//    return 0;
//
//}

