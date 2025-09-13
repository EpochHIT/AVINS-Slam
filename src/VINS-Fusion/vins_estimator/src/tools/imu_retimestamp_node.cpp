#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuRestamper {
public:
  ImuRestamper(ros::NodeHandle& pnh) {
    pnh.param<std::string>("in", in_topic_, std::string("/camera/imu"));
    pnh.param<std::string>("out", out_topic_, std::string("/camera/imu_sync"));
    int qsize = 1000;
    pnh.param<int>("queue_size", qsize, 1000);
    sub_ = nh_.subscribe(in_topic_, qsize, &ImuRestamper::cb, this, ros::TransportHints().tcpNoDelay());
    pub_ = nh_.advertise<sensor_msgs::Imu>(out_topic_, qsize);
    ROS_INFO_STREAM("imu_retimestamp: subscribing " << in_topic_ << ", publishing " << out_topic_);
  }

private:
  void cb(const sensor_msgs::ImuConstPtr& msg) {
    sensor_msgs::Imu out = *msg;
    out.header.stamp = ros::Time::now();
    pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string in_topic_;
  std::string out_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_retimestamp_node");
  ros::NodeHandle pnh("~");
  ImuRestamper node(pnh);
  ros::spin();
  return 0;
}
