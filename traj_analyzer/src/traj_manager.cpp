#include <ros/ros.h>
#include <traj_analyzer/RefTraj.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <deque>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>

class TrajectoryManager {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber current_pose_sub_;
    ros::Publisher cmd_ref_traj_pub_;  // Publish command reference trajectory
    ros::Publisher lateral_ref_pub_;   // Publish lateral reference point
    ros::Publisher nearest_idx_pub_;   // Publish the index of the nearest point
    ros::Publisher nearest_dist_pub_;  // 发布最近点的实际距离
    
    std::vector<traj_analyzer::RefTraj> all_trajectory_points_; // Store the entire trajectory file
    std::deque<traj_analyzer::RefTraj> fifo_buffer_;  // 20-line FIFO buffer
    geometry_msgs::PoseStamped current_pose_;
    bool pose_received_;
    
    int fifo_size_;        // FIFO buffer size
    int current_traj_idx_; // Current trajectory index
    std::string traj_file_path_; // Trajectory file path
    std::string frame_id_;  // Frame ID for messages
    double publish_rate_;  // Publishing rate in Hz
    
    ros::Timer update_timer_; // Timer for updates
    
public:
    TrajectoryManager() : private_nh_("~") {
        // Read parameters from ROS parameter server
        private_nh_.param<int>("fifo_size", fifo_size_, 200);
        private_nh_.param<std::string>("traj_file_path", traj_file_path_, "traj_analyzer/refine_traj_fast.txt");
        private_nh_.param<double>("publish_rate", publish_rate_, 20.0); // Default 20Hz
        private_nh_.param<std::string>("frame_id", frame_id_, "odom_capture_frame");
        
        pose_received_ = false;
        current_traj_idx_ = 0;
        
        // Publish command reference trajectory and lateral reference point
        cmd_ref_traj_pub_ = nh_.advertise<traj_analyzer::RefTraj>("cmd_ref_trajectory", 10);
        lateral_ref_pub_ = nh_.advertise<traj_analyzer::RefTraj>("lateral_ref", 10);
        nearest_idx_pub_ = nh_.advertise<std_msgs::Int32>("nearest_traj_idx", 10);
        nearest_dist_pub_ = nh_.advertise<std_msgs::Float64>("nearest_point_distance", 10);
        
        // Subscribe to odometry/filtered
        current_pose_sub_ = nh_.subscribe("odometry/filtered", 10, &TrajectoryManager::odomCallback, this);
        
        // Load trajectory file
        loadTrajectoryFile();
        
        // Initialize FIFO buffer
        initializeFIFO();
        
        // Create timer with the specified publish rate
        double update_period = 1.0 / publish_rate_;
        update_timer_ = nh_.createTimer(ros::Duration(update_period), &TrajectoryManager::updateCallback, this);
        
        ROS_INFO("Trajectory manager initialized with publish rate: %.1f Hz", publish_rate_);
    }
    

    // Add new callback function to handle Odometry message
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Convert Odometry message to PoseStamped format
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;
        pose_received_ = true;
    }
    
    void loadTrajectoryFile() {
        // Load the entire trajectory file into memory at once
        std::ifstream file(traj_file_path_);
        if (!file.is_open()) {
            ROS_ERROR("Cannot open trajectory file: %s", traj_file_path_.c_str());
            return;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            // Skip empty lines
            if (line.empty()) continue;
            
            std::istringstream iss(line);
            double x, y, psi, v, curvature, s, d;
            
            if (iss >> x >> y >> psi >> v >> curvature >> s >> d) {
                traj_analyzer::RefTraj point;
                point.header.stamp = ros::Time::now();
                point.header.frame_id = frame_id_;
                point.x = x;
                point.y = y;
                point.psi = psi;
                point.v = v;
                point.curvature = curvature;
                point.s = s;
                point.d = d;
                
                all_trajectory_points_.push_back(point);
            }
        }
        
        file.close();
        ROS_INFO("Successfully loaded %zu trajectory points", all_trajectory_points_.size());
    }
    
    void initializeFIFO() {
        // Clear FIFO
        fifo_buffer_.clear();
        
        // Fill the first 110 lines with zeros (velocity 0.6)
        for (int i = 0; i < (fifo_size_ / 2+10); i++) {
            traj_analyzer::RefTraj zero_point;
            zero_point.header.stamp = ros::Time::now();
            zero_point.header.frame_id = frame_id_;
            zero_point.x = 0.0;
            zero_point.y = 0.0;
            zero_point.psi = 0.0;
            zero_point.v = 0.6;  // Velocity is 0.6
            zero_point.curvature = 0.0;
            zero_point.s = 0.0;  // 初始化s坐标为0
            zero_point.d = 0.0;  // 初始化d坐标为0
            
            fifo_buffer_.push_back(zero_point);
        }
        
        // Only fill 90 points, leave 10 points for the first updateCallback
        int points_to_add = std::min(fifo_size_ / 2 - 10, static_cast<int>(all_trajectory_points_.size()));
        for (int i = 0; i < points_to_add; i++) {
            fifo_buffer_.push_back(all_trajectory_points_[i]);
        }
        
        current_traj_idx_ = points_to_add;
    }
    
    void updateCallback(const ros::TimerEvent& event) {
        if (all_trajectory_points_.empty()) {
            ROS_WARN("Trajectory points are empty, cannot update FIFO");
            return;
        }
        
        // 计算FIFO中间点的索引
        int middle_idx = fifo_size_ / 2;
        
        // Check if the trajectory has been updated
        if (current_traj_idx_ >= all_trajectory_points_.size() && 
            fifo_buffer_.size() > middle_idx && 
            fifo_buffer_[middle_idx].x == all_trajectory_points_.back().x && 
            fifo_buffer_[middle_idx].y == all_trajectory_points_.back().y) {
            ROS_INFO_ONCE("Reached the end of trajectory and middle point is the last trajectory point, stopping updates");
            return;  // When the middle point is the last trajectory point, stop updating
        }
        
        // Remove points from the front of FIFO
        int points_to_remove = std::min(10, static_cast<int>(fifo_buffer_.size()));
        for (int i = 0; i < points_to_remove; i++) {
            if (!fifo_buffer_.empty()) {
                fifo_buffer_.pop_front();
            }
        }
        
        // Add new trajectory points to FIFO
        int points_to_add = 10;  // Always add 10 points
        for (int i = 0; i < points_to_add; i++) {
            if (current_traj_idx_ < all_trajectory_points_.size()) {
                // If there are still trajectory points, add the next trajectory point
                fifo_buffer_.push_back(all_trajectory_points_[current_traj_idx_++]);
            } else {
                // If the trajectory points are used up, repeat the last trajectory point
                fifo_buffer_.push_back(all_trajectory_points_.back());
            }
        }
        
        // Publish cmd_ref_trajectory (middle line of FIFO)
        if (fifo_buffer_.size() > middle_idx) {
            cmd_ref_traj_pub_.publish(fifo_buffer_[middle_idx]);
        }
        
        // If current position is received, find and publish the nearest point
        if (pose_received_) {
            double min_distance;  // 用于存储最小距离
            int nearest_idx = findNearestPointInFIFO(&min_distance);  // 修改调用，传入距离变量的指针
            if (nearest_idx >= 0 && nearest_idx < fifo_buffer_.size()) {
                lateral_ref_pub_.publish(fifo_buffer_[nearest_idx]);
                
                // Publish the index of the nearest point
                std_msgs::Int32 idx_msg;
                idx_msg.data = nearest_idx;
                nearest_idx_pub_.publish(idx_msg);
                
                // 发布最近点的实际距离
                std_msgs::Float64 dist_msg;
                dist_msg.data = min_distance;
                nearest_dist_pub_.publish(dist_msg);
            }
        }
    }
    
    int findNearestPointInFIFO(double* min_distance = nullptr) {
        if (fifo_buffer_.empty()) {
            return -1;
        }
        
        double min_dist = std::numeric_limits<double>::max();
        int nearest_idx = 0;
        
        double current_x = current_pose_.pose.position.x;
        double current_y = current_pose_.pose.position.y;
        
        for (int i = 0; i < fifo_buffer_.size(); i++) {
            double dx = fifo_buffer_[i].x - current_x;
            double dy = fifo_buffer_[i].y - current_y;
            double dist = dx * dx + dy * dy;  // Squared distance, no need for square root
            
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        // 计算实际距离（取平方根）并通过指针返回
        if (min_distance != nullptr) {
            *min_distance = std::sqrt(min_dist);
        }
        
        return nearest_idx;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_manager");
    TrajectoryManager manager;
    ros::spin();
    return 0;
}