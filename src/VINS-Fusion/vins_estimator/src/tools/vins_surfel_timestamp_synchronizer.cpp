#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <mutex>
#include <algorithm>

class VINSSurfelTimestampSynchronizer {
private:
    ros::NodeHandle nh_;
    
    // 订阅者
    ros::Subscriber sub_color_;
    ros::Subscriber sub_depth_;
    ros::Subscriber sub_infra1_;
    ros::Subscriber sub_infra2_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_vins_odom_;
    
    // 发布者
    ros::Publisher pub_color_sync_;
    ros::Publisher pub_depth_sync_;
    ros::Publisher pub_infra1_sync_;
    ros::Publisher pub_infra2_sync_;
    ros::Publisher pub_imu_sync_;
    
    // 数据缓冲区
    std::deque<std::pair<ros::Time, sensor_msgs::ImageConstPtr>> color_buffer_;
    std::deque<std::pair<ros::Time, sensor_msgs::ImageConstPtr>> depth_buffer_;
    std::deque<std::pair<ros::Time, sensor_msgs::ImageConstPtr>> infra1_buffer_;
    std::deque<std::pair<ros::Time, sensor_msgs::ImageConstPtr>> infra2_buffer_;
    std::deque<std::pair<ros::Time, sensor_msgs::ImuConstPtr>> imu_buffer_;
    std::deque<std::pair<ros::Time, nav_msgs::OdometryConstPtr>> vins_buffer_;
    
    std::mutex buffer_mutex_;
    
    // 参数
    double sync_tolerance_;
    double imu_sync_tolerance_;
    int max_buffer_size_;
    bool enable_color_;
    bool enable_depth_;
    bool enable_infra_;
    bool enable_imu_sync_;
    
    ros::Time last_sync_time_;
    
public:
    VINSSurfelTimestampSynchronizer() : nh_("~") {
        // 获取参数
        nh_.param<double>("sync_tolerance", sync_tolerance_, 0.01);      // 10ms容错 for images
        nh_.param<double>("imu_sync_tolerance", imu_sync_tolerance_, 0.005); // 5ms容错 for IMU
        nh_.param<int>("max_buffer_size", max_buffer_size_, 200);
        nh_.param<bool>("enable_color", enable_color_, true);
        nh_.param<bool>("enable_depth", enable_depth_, true);
        nh_.param<bool>("enable_infra", enable_infra_, true);
        nh_.param<bool>("enable_imu_sync", enable_imu_sync_, true);
        
        // 订阅原始数据
        if (enable_color_) {
            sub_color_ = nh_.subscribe("/camera/color/image_raw", 50, 
                                       &VINSSurfelTimestampSynchronizer::colorCallback, this);
            pub_color_sync_ = nh_.advertise<sensor_msgs::Image>("/camera/color/image_raw_sync", 50);
        }
        
        if (enable_depth_) {
            sub_depth_ = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 50, 
                                       &VINSSurfelTimestampSynchronizer::depthCallback, this);
            pub_depth_sync_ = nh_.advertise<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw_sync", 50);
        }
        
        if (enable_infra_) {
            sub_infra1_ = nh_.subscribe("/camera/infra1/image_rect_raw", 50, 
                                        &VINSSurfelTimestampSynchronizer::infra1Callback, this);
            sub_infra2_ = nh_.subscribe("/camera/infra2/image_rect_raw", 50, 
                                        &VINSSurfelTimestampSynchronizer::infra2Callback, this);
            pub_infra1_sync_ = nh_.advertise<sensor_msgs::Image>("/camera/infra1/image_rect_raw_sync", 50);
            pub_infra2_sync_ = nh_.advertise<sensor_msgs::Image>("/camera/infra2/image_rect_raw_sync", 50);
        }
        
        if (enable_imu_sync_) {
            sub_imu_ = nh_.subscribe("/camera/imu", 200, 
                                     &VINSSurfelTimestampSynchronizer::imuCallback, this);
            pub_imu_sync_ = nh_.advertise<sensor_msgs::Imu>("/camera/imu_sync", 200);
        }
        
        // 订阅VINS里程计作为时间戳参考
        sub_vins_odom_ = nh_.subscribe("/vins_estimator/odometry", 100, 
                                       &VINSSurfelTimestampSynchronizer::vinsOdomCallback, this);
        
        last_sync_time_ = ros::Time(0);
        
        ROS_INFO("VINS-Surfel Timestamp Synchronizer initialized");
        ROS_INFO("Image sync tolerance: %.3f seconds", sync_tolerance_);
        ROS_INFO("IMU sync tolerance: %.3f seconds", imu_sync_tolerance_);
        ROS_INFO("Enable - Color: %s, Depth: %s, Infra: %s, IMU: %s", 
                 enable_color_ ? "true" : "false",
                 enable_depth_ ? "true" : "false", 
                 enable_infra_ ? "true" : "false",
                 enable_imu_sync_ ? "true" : "false");
    }
    
    void colorCallback(const sensor_msgs::ImageConstPtr& msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        color_buffer_.push_back(std::make_pair(msg->header.stamp, msg));
        limitBufferSize(color_buffer_);
        synchronizeData();
    }
    
    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        depth_buffer_.push_back(std::make_pair(msg->header.stamp, msg));
        limitBufferSize(depth_buffer_);
        synchronizeData();
    }
    
    void infra1Callback(const sensor_msgs::ImageConstPtr& msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        infra1_buffer_.push_back(std::make_pair(msg->header.stamp, msg));
        limitBufferSize(infra1_buffer_);
        synchronizeData();
    }
    
    void infra2Callback(const sensor_msgs::ImageConstPtr& msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        infra2_buffer_.push_back(std::make_pair(msg->header.stamp, msg));
        limitBufferSize(infra2_buffer_);
        synchronizeData();
    }
    
    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        if (!enable_imu_sync_) return;
        
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        imu_buffer_.push_back(std::make_pair(msg->header.stamp, msg));
        limitBufferSize(imu_buffer_);
        synchronizeIMU();
    }
    
    void vinsOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        vins_buffer_.push_back(std::make_pair(msg->header.stamp, msg));
        limitBufferSize(vins_buffer_);
        synchronizeData();
    }
    
    template<typename T>
    void limitBufferSize(std::deque<std::pair<ros::Time, T>>& buffer) {
        while (buffer.size() > max_buffer_size_) {
            buffer.pop_front();
        }
    }
    
    void synchronizeData() {
        if (vins_buffer_.empty()) return;
        
        // 遍历VINS位姿，寻找匹配的图像
        for (auto vins_it = vins_buffer_.begin(); vins_it != vins_buffer_.end(); ) {
            ros::Time vins_time = vins_it->first;
            
            // 避免重复处理相同时间戳
            if (vins_time <= last_sync_time_) {
                vins_it = vins_buffer_.erase(vins_it);
                continue;
            }
            
            bool found_match = true;
            
            // 查找匹配的图像
            auto color_match = findClosestMatch(color_buffer_, vins_time, sync_tolerance_);
            auto depth_match = findClosestMatch(depth_buffer_, vins_time, sync_tolerance_);
            auto infra1_match = findClosestMatch(infra1_buffer_, vins_time, sync_tolerance_);
            auto infra2_match = findClosestMatch(infra2_buffer_, vins_time, sync_tolerance_);
            
            // 检查必要的数据是否都找到了
            if (enable_color_ && color_match == color_buffer_.end()) found_match = false;
            if (enable_depth_ && depth_match == depth_buffer_.end()) found_match = false;
            if (enable_infra_ && (infra1_match == infra1_buffer_.end() || infra2_match == infra2_buffer_.end())) found_match = false;
            
            if (found_match) {
                // 发布同步后的数据
                publishSynchronizedData(vins_time, color_match, depth_match, infra1_match, infra2_match);
                
                // 清理已使用的数据
                cleanUsedData(color_match, depth_match, infra1_match, infra2_match);
                
                last_sync_time_ = vins_time;
                vins_it = vins_buffer_.erase(vins_it);
                
                ROS_DEBUG("Synchronized at VINS time: %.6f", vins_time.toSec());
            } else {
                ++vins_it;
            }
        }
        
        // 清理过旧的数据
        cleanOldData();
    }
    
    void synchronizeIMU() {
        if (!enable_imu_sync_ || vins_buffer_.empty() || imu_buffer_.empty()) return;
        
        // 为每个VINS时间戳寻找最接近的IMU数据
        for (auto& vins_pair : vins_buffer_) {
            ros::Time vins_time = vins_pair.first;
            
            auto imu_match = findClosestMatch(imu_buffer_, vins_time, imu_sync_tolerance_);
            if (imu_match != imu_buffer_.end()) {
                // 发布时间戳同步的IMU数据
                sensor_msgs::Imu imu_sync = *imu_match->second;
                imu_sync.header.stamp = vins_time;
                pub_imu_sync_.publish(imu_sync);
                
                // 不在这里删除IMU数据，因为一个IMU数据可能对应多个VINS时间戳
            }
        }
    }
    
    template<typename T>
    typename std::deque<std::pair<ros::Time, T>>::iterator 
    findClosestMatch(std::deque<std::pair<ros::Time, T>>& buffer, 
                     const ros::Time& target_time, double tolerance) {
        if (buffer.empty()) return buffer.end();
        
        auto best_it = buffer.end();
        double min_diff = std::numeric_limits<double>::max();
        
        for (auto it = buffer.begin(); it != buffer.end(); ++it) {
            double time_diff = fabs((it->first - target_time).toSec());
            if (time_diff < min_diff && time_diff < tolerance) {
                min_diff = time_diff;
                best_it = it;
            }
        }
        
        return best_it;
    }
    
    void publishSynchronizedData(const ros::Time& sync_time,
                                const decltype(color_buffer_)::iterator& color_it,
                                const decltype(depth_buffer_)::iterator& depth_it,
                                const decltype(infra1_buffer_)::iterator& infra1_it,
                                const decltype(infra2_buffer_)::iterator& infra2_it) {
        
        if (enable_color_ && color_it != color_buffer_.end()) {
            sensor_msgs::Image color_sync = *color_it->second;
            color_sync.header.stamp = sync_time;
            pub_color_sync_.publish(color_sync);
        }
        
        if (enable_depth_ && depth_it != depth_buffer_.end()) {
            sensor_msgs::Image depth_sync = *depth_it->second;
            depth_sync.header.stamp = sync_time;
            pub_depth_sync_.publish(depth_sync);
        }
        
        if (enable_infra_ && infra1_it != infra1_buffer_.end() && infra2_it != infra2_buffer_.end()) {
            sensor_msgs::Image infra1_sync = *infra1_it->second;
            sensor_msgs::Image infra2_sync = *infra2_it->second;
            
            infra1_sync.header.stamp = sync_time;
            infra2_sync.header.stamp = sync_time;
            
            pub_infra1_sync_.publish(infra1_sync);
            pub_infra2_sync_.publish(infra2_sync);
        }
    }
    
    void cleanUsedData(const decltype(color_buffer_)::iterator& color_it,
                       const decltype(depth_buffer_)::iterator& depth_it,
                       const decltype(infra1_buffer_)::iterator& infra1_it,
                       const decltype(infra2_buffer_)::iterator& infra2_it) {
        
        if (enable_color_ && color_it != color_buffer_.end()) {
            color_buffer_.erase(color_buffer_.begin(), color_it + 1);
        }
        
        if (enable_depth_ && depth_it != depth_buffer_.end()) {
            depth_buffer_.erase(depth_buffer_.begin(), depth_it + 1);
        }
        
        if (enable_infra_) {
            if (infra1_it != infra1_buffer_.end()) {
                infra1_buffer_.erase(infra1_buffer_.begin(), infra1_it + 1);
            }
            if (infra2_it != infra2_buffer_.end()) {
                infra2_buffer_.erase(infra2_buffer_.begin(), infra2_it + 1);
            }
        }
    }
    
    void cleanOldData() {
        ros::Time current_time = ros::Time::now();
        double max_age = 2.0;  // 清理2秒前的数据
        
        auto cleanBuffer = [&](auto& buffer) {
            while (!buffer.empty() && 
                   (current_time - buffer.front().first).toSec() > max_age) {
                buffer.pop_front();
            }
        };
        
        cleanBuffer(color_buffer_);
        cleanBuffer(depth_buffer_);
        cleanBuffer(infra1_buffer_);
        cleanBuffer(infra2_buffer_);
        cleanBuffer(imu_buffer_);
        cleanBuffer(vins_buffer_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vins_surfel_timestamp_synchronizer");
    
    VINSSurfelTimestampSynchronizer synchronizer;
    
    ros::Rate rate(200);  // 200Hz for high-frequency processing
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
