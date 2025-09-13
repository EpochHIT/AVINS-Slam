#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <mutex>

class ImageTimestampSynchronizer {
private:
    ros::NodeHandle nh_;
    
    // 订阅者
    ros::Subscriber sub_color_;
    ros::Subscriber sub_depth_;
    ros::Subscriber sub_vins_path_;
    
    // 发布者
    ros::Publisher pub_color_sync_;
    ros::Publisher pub_depth_sync_;
    
    // 数据缓冲区
    std::deque<std::pair<ros::Time, sensor_msgs::ImageConstPtr>> color_buffer_;
    std::deque<std::pair<ros::Time, sensor_msgs::ImageConstPtr>> depth_buffer_;
    std::deque<std::pair<ros::Time, geometry_msgs::PoseStamped>> pose_buffer_;
    
    std::mutex buffer_mutex_;
    
    // 参数
    double sync_tolerance_;
    int max_buffer_size_;
    
public:
    ImageTimestampSynchronizer() : nh_("~") {
        // 获取参数
        nh_.param<double>("sync_tolerance", sync_tolerance_, 0.05);  // 50ms容错
        nh_.param<int>("max_buffer_size", max_buffer_size_, 100);
        
        // 订阅原始数据
        sub_color_ = nh_.subscribe("/camera_rgbd/color/image_raw", 10, 
                                   &ImageTimestampSynchronizer::colorCallback, this);
        sub_depth_ = nh_.subscribe("/camera_rgbd/aligned_depth_to_color/image_raw", 10, 
                                   &ImageTimestampSynchronizer::depthCallback, this);
        sub_vins_path_ = nh_.subscribe("/vins_estimator/path", 10, 
                                       &ImageTimestampSynchronizer::vinsPathCallback, this);
        
        // 发布同步后的数据
        pub_color_sync_ = nh_.advertise<sensor_msgs::Image>("/camera_rgbd/color/image_raw_sync", 10);
        pub_depth_sync_ = nh_.advertise<sensor_msgs::Image>("/camera_rgbd/aligned_depth_to_color/image_raw_sync", 10);
        
        ROS_INFO("Image Timestamp Synchronizer initialized");
        ROS_INFO("Sync tolerance: %.3f seconds", sync_tolerance_);
    }
    
    void colorCallback(const sensor_msgs::ImageConstPtr& msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        color_buffer_.push_back(std::make_pair(msg->header.stamp, msg));
        
        // 限制缓冲区大小
        if (color_buffer_.size() > max_buffer_size_) {
            color_buffer_.pop_front();
        }
        
        synchronizeData();
    }
    
    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        depth_buffer_.push_back(std::make_pair(msg->header.stamp, msg));
        
        // 限制缓冲区大小
        if (depth_buffer_.size() > max_buffer_size_) {
            depth_buffer_.pop_front();
        }
        
        synchronizeData();
    }
    
    void vinsPathCallback(const nav_msgs::PathConstPtr& msg) {
        if (msg->poses.empty()) return;
        
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        
        // 添加最新的位姿到缓冲区
        geometry_msgs::PoseStamped latest_pose = msg->poses.back();
        pose_buffer_.push_back(std::make_pair(latest_pose.header.stamp, latest_pose));
        
        // 限制缓冲区大小
        if (pose_buffer_.size() > max_buffer_size_) {
            pose_buffer_.pop_front();
        }
        
        synchronizeData();
    }
    
    void synchronizeData() {
        if (pose_buffer_.empty() || color_buffer_.empty() || depth_buffer_.empty()) {
            return;
        }
        
        // 遍历每个位姿时间戳，寻找匹配的图像
        for (auto pose_it = pose_buffer_.begin(); pose_it != pose_buffer_.end(); ) {
            ros::Time pose_time = pose_it->first;
            geometry_msgs::PoseStamped pose = pose_it->second;
            
            // 寻找时间戳最接近的彩色图像
            auto best_color_it = color_buffer_.end();
            double min_color_diff = std::numeric_limits<double>::max();
            
            for (auto color_it = color_buffer_.begin(); color_it != color_buffer_.end(); ++color_it) {
                double time_diff = fabs((color_it->first - pose_time).toSec());
                if (time_diff < min_color_diff) {
                    min_color_diff = time_diff;
                    best_color_it = color_it;
                }
            }
            
            // 寻找时间戳最接近的深度图像
            auto best_depth_it = depth_buffer_.end();
            double min_depth_diff = std::numeric_limits<double>::max();
            
            for (auto depth_it = depth_buffer_.begin(); depth_it != depth_buffer_.end(); ++depth_it) {
                double time_diff = fabs((depth_it->first - pose_time).toSec());
                if (time_diff < min_depth_diff) {
                    min_depth_diff = time_diff;
                    best_depth_it = depth_it;
                }
            }
            
            // 检查是否在容错范围内
            if (best_color_it != color_buffer_.end() && 
                best_depth_it != depth_buffer_.end() &&
                min_color_diff < sync_tolerance_ && 
                min_depth_diff < sync_tolerance_) {
                
                // 发布时间戳同步后的图像
                publishSynchronizedImages(pose_time, best_color_it->second, best_depth_it->second);
                
                // 清理已使用的数据
                color_buffer_.erase(color_buffer_.begin(), best_color_it + 1);
                depth_buffer_.erase(depth_buffer_.begin(), best_depth_it + 1);
                pose_it = pose_buffer_.erase(pose_it);
                
                ROS_DEBUG("Synchronized at time: %.6f, color_diff: %.6f, depth_diff: %.6f", 
                         pose_time.toSec(), min_color_diff, min_depth_diff);
            } else {
                ++pose_it;
            }
        }
        
        // 清理过旧的数据
        cleanOldData();
    }
    
    void publishSynchronizedImages(const ros::Time& sync_time, 
                                   const sensor_msgs::ImageConstPtr& color_msg,
                                   const sensor_msgs::ImageConstPtr& depth_msg) {
        // 创建新的消息，更新时间戳
        sensor_msgs::Image color_sync = *color_msg;
        sensor_msgs::Image depth_sync = *depth_msg;
        
        color_sync.header.stamp = sync_time;
        depth_sync.header.stamp = sync_time;
        
        pub_color_sync_.publish(color_sync);
        pub_depth_sync_.publish(depth_sync);
    }
    
    void cleanOldData() {
        ros::Time current_time = ros::Time::now();
        double max_age = 1.0;  // 清理1秒前的数据
        
        // 清理旧的彩色图像
        while (!color_buffer_.empty() && 
               (current_time - color_buffer_.front().first).toSec() > max_age) {
            color_buffer_.pop_front();
        }
        
        // 清理旧的深度图像
        while (!depth_buffer_.empty() && 
               (current_time - depth_buffer_.front().first).toSec() > max_age) {
            depth_buffer_.pop_front();
        }
        
        // 清理旧的位姿
        while (!pose_buffer_.empty() && 
               (current_time - pose_buffer_.front().first).toSec() > max_age) {
            pose_buffer_.pop_front();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_timestamp_synchronizer");
    
    ImageTimestampSynchronizer synchronizer;
    
    ros::Rate rate(100);  // 100Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
