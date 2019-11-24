#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <vector>

#include "hark_msgs/HarkWave.h"
#include "hark_msgs/HarkWaveVal.h"


class SendWav
{
    private:
        std::vector<float> arr;
        hark_msgs::HarkWave hw;
        hark_msgs::HarkWaveVal hwv;
        int count = 0;
        bool sent_flag = false;
    public:
        SendWav();
        void callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
        void publish_to_hark(std::vector<float> arr);
        ros::NodeHandle n;
        ros::Publisher HarkWav_pub = n.advertise<hark_msgs::HarkWave>("HarkWav", 10);
};

SendWav::SendWav()
{
    std::cout << "SendWav initialized." << std::endl;
    hw.length = 512;
    hw.nch = 1;
    hw.data_bytes = hw.length * 4;
}

void SendWav::callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    arr = {};
    for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    {
      arr.push_back((float)*it);
    }
    std::cout << arr[0] << std::endl;
    if (arr.size() == hw.length)
    {
        SendWav::publish_to_hark(arr);
        sent_flag = true;
    }
    else
    {
        if (sent_flag) exit(1);
        std::cout << "not sending to HARK now" << std::endl;
    }
}

void SendWav::publish_to_hark(std::vector<float> arr)
{
    ros::Time t = ros::Time::now();
    hw.header.stamp = t;
    hw.header.frame_id = std::to_string(count);
    hw.count = count;
    hwv.wavedata = arr;
    hw.src = {};
    hw.src.push_back(hwv);
    HarkWav_pub.publish(hw);
    count++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wav_reader_cpp");
    ros::NodeHandle n;
    SendWav sw;
    ros::Subscriber sub0 = n.subscribe("wav_ch0", 100000, &SendWav::callback, &sw);
//    ros::Subscriber sub1 = n.subscribe("wav_ch1", 10, &SendWav::callback, &sw);
//    ros::Subscriber sub2 = n.subscribe("wav_ch2", 10, &SendWav::callback, &sw);
//    ros::Subscriber sub3 = n.subscribe("wav_ch3", 10, &SendWav::callback, &sw);
//    ros::Subscriber sub4 = n.subscribe("wav_ch4", 10, &SendWav::callback, &sw);
//    ros::Subscriber sub5 = n.subscribe("wav_ch5", 10, &SendWav::callback, &sw);
//    ros::Subscriber sub6 = n.subscribe("wav_ch6", 10, &SendWav::callback, &sw);
//    ros::Subscriber sub7 = n.subscribe("wav_ch7", 10, &SendWav::callback, &sw);
    ROS_INFO("Waiting for message ...");
    ros::spin();
    return 0;
}
