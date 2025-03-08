#include <ros/ros.h>
#include <traj_analyzer/RefTraj.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <cmath>
#include <std_srvs/Empty.h>

class TrajectoryVisualizer {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_; // Private node handle for parameters
    ros::Subscriber cmd_ref_traj_sub_;
    ros::Subscriber lateral_ref_sub_;
    ros::Publisher cmd_traj_marker_pub_;
    ros::Publisher lateral_ref_marker_pub_;
    ros::Publisher velocity_marker_pub_;  // New publisher for velocity arrows
    ros::Timer timer_;
    
    traj_analyzer::RefTraj latest_cmd_traj_;
    traj_analyzer::RefTraj latest_lateral_ref_;
    std::vector<traj_analyzer::RefTraj> cmd_traj_points_;
    bool cmd_traj_received_;
    bool lateral_ref_received_;
    
    std::string frame_id_;
    double point_size_;
    double line_width_;
    double arrow_scale_;  // Scale factor for velocity arrows
    int traj_display_length_;  // Parameter for trajectory display length
    
    ros::ServiceServer clear_traj_service_;
    
    // Ensure angle change is continuous, avoid jumps near 0° and 360°
    double normalizeAngleChange(double prev_angle, double current_angle) {
        // Convert angle to [-π, π] range
        double normalized_prev = fmod(prev_angle, 2.0 * M_PI);
        if (normalized_prev > M_PI) normalized_prev -= 2.0 * M_PI;
        if (normalized_prev < -M_PI) normalized_prev += 2.0 * M_PI;
        
        double normalized_current = fmod(current_angle, 2.0 * M_PI);
        if (normalized_current > M_PI) normalized_current -= 2.0 * M_PI;
        if (normalized_current < -M_PI) normalized_current += 2.0 * M_PI;
        
        // Calculate minimum angle change
        double delta = normalized_current - normalized_prev;
        if (delta > M_PI) delta -= 2.0 * M_PI;
        if (delta < -M_PI) delta += 2.0 * M_PI;
        
        // Return the continuous angle
        return normalized_prev + delta;
    }
    
public:
    TrajectoryVisualizer() : private_nh_("~") { // Initialize private node handle with "~"
        // Get parameters from the private namespace
        private_nh_.param<std::string>("frame_id", frame_id_, "odom_capture_frame");
        private_nh_.param<double>("point_size", point_size_, 0.2);
        private_nh_.param<double>("line_width", line_width_, 0.05);
        private_nh_.param<double>("arrow_scale", arrow_scale_, 0.5);
        private_nh_.param<int>("traj_display_length", traj_display_length_, 100);
        
        // Print all parameters for debugging
        ROS_INFO("Parameters:");
        ROS_INFO("  frame_id: %s", frame_id_.c_str());
        ROS_INFO("  point_size: %.2f", point_size_);
        ROS_INFO("  line_width: %.2f", line_width_);
        ROS_INFO("  arrow_scale: %.2f", arrow_scale_);
        ROS_INFO("  traj_display_length: %d", traj_display_length_);
        
        // Check if the parameter exists in the parameter server
        int global_traj_display_length;
        if (nh_.getParam("/traj_display_length", global_traj_display_length)) {
            ROS_INFO("Found global traj_display_length: %d", global_traj_display_length);
        }
        
        int private_traj_display_length;
        if (private_nh_.getParam("traj_display_length", private_traj_display_length)) {
            ROS_INFO("Found private traj_display_length: %d", private_traj_display_length);
        }
        
        cmd_traj_received_ = false;
        lateral_ref_received_ = false;
        
        // Publishers for visualization markers
        cmd_traj_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("cmd_traj_markers", 1);
        lateral_ref_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("lateral_ref_marker", 1);
        velocity_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("velocity_markers", 1);  // New publisher
        
        // Subscribers
        cmd_ref_traj_sub_ = nh_.subscribe("cmd_ref_trajectory", 100, &TrajectoryVisualizer::cmdTrajCallback, this);
        lateral_ref_sub_ = nh_.subscribe("lateral_ref", 10, &TrajectoryVisualizer::lateralRefCallback, this);
        
        // Timer for visualization update
        timer_ = nh_.createTimer(ros::Duration(0.1), &TrajectoryVisualizer::timerCallback, this);
        
        // 添加清除轨迹的服务
        clear_traj_service_ = nh_.advertiseService("clear_trajectory", 
                                                  &TrajectoryVisualizer::clearTrajectoryCallback, this);
        
        ROS_INFO("Trajectory visualizer initialized with display length: %d points", traj_display_length_);
    }
    
    void cmdTrajCallback(const traj_analyzer::RefTraj::ConstPtr& msg) {
        latest_cmd_traj_ = *msg;
        cmd_traj_received_ = true;
        
        // Store command trajectory points (with a configurable maximum buffer size)
        cmd_traj_points_.push_back(*msg);
        while (cmd_traj_points_.size() > traj_display_length_) {
            cmd_traj_points_.erase(cmd_traj_points_.begin());
        }
        
        // Periodically log the buffer size for debugging
        static int count = 0;
        if (++count % 100 == 0) {
            ROS_INFO("Current trajectory buffer size: %zu/%d", cmd_traj_points_.size(), traj_display_length_);
        }
    }
    
    void lateralRefCallback(const traj_analyzer::RefTraj::ConstPtr& msg) {
        latest_lateral_ref_ = *msg;
        lateral_ref_received_ = true;
    }
    
    void timerCallback(const ros::TimerEvent&) {
        // Publish visualization markers
        publishCmdTrajectoryMarker();
        publishLateralRefMarker();
        publishVelocityMarkers();  // New function to publish velocity arrows
    }
    
    void publishCmdTrajectoryMarker() {
        if (!cmd_traj_received_ || cmd_traj_points_.empty()) return;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "cmd_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = line_width_;  // line width
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.5);  // short lifetime to auto-clear old points
        
        for (const auto& point : cmd_traj_points_) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.05;  // slightly above ground
            marker.points.push_back(p);
        }
        
        cmd_traj_marker_pub_.publish(marker);
        
        // Also publish current command point as a sphere
        visualization_msgs::Marker current_point_marker;
        current_point_marker.header = marker.header;
        current_point_marker.ns = "current_cmd_point";
        current_point_marker.id = 2;
        current_point_marker.type = visualization_msgs::Marker::SPHERE;
        current_point_marker.action = visualization_msgs::Marker::ADD;
        current_point_marker.pose.position.x = latest_cmd_traj_.x;
        current_point_marker.pose.position.y = latest_cmd_traj_.y;
        current_point_marker.pose.position.z = 0.1;
        current_point_marker.pose.orientation.w = 1.0;
        current_point_marker.scale.x = current_point_marker.scale.y = current_point_marker.scale.z = point_size_ * 1.5;
        current_point_marker.color.r = 0.0;
        current_point_marker.color.g = 1.0;
        current_point_marker.color.b = 0.0;
        current_point_marker.color.a = 1.0;
        current_point_marker.lifetime = ros::Duration(0.5);
        
        cmd_traj_marker_pub_.publish(current_point_marker);
    }
    
    void publishLateralRefMarker() {
        if (!lateral_ref_received_) return;
        
        // Publish lateral reference point as a sphere
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "lateral_ref";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = latest_lateral_ref_.x;
        marker.pose.position.y = latest_lateral_ref_.y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = point_size_ * 2.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;  // Purple for lateral reference
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.5);
        
        lateral_ref_marker_pub_.publish(marker);
    }
    
    void publishVelocityMarkers() {
        if (!cmd_traj_received_) return;
        
        // Only create an arrow for the current command point
        visualization_msgs::Marker current_vel_marker;
        current_vel_marker.header.frame_id = frame_id_;
        current_vel_marker.header.stamp = ros::Time::now();
        current_vel_marker.ns = "current_velocity";
        current_vel_marker.id = 0;
        current_vel_marker.type = visualization_msgs::Marker::ARROW;
        current_vel_marker.action = visualization_msgs::Marker::ADD;
        current_vel_marker.lifetime = ros::Duration(0.5);
        
        // Set color to blue for velocity
        current_vel_marker.color.r = 0.0;
        current_vel_marker.color.g = 0.0;
        current_vel_marker.color.b = 1.0;
        current_vel_marker.color.a = 1.0;
        
        // Set arrow properties
        current_vel_marker.scale.x = 0.08;  // Shaft diameter
        current_vel_marker.scale.y = 0.15;  // Head diameter
        current_vel_marker.scale.z = 0.3;   // Head length
        
        // Convert psi from degrees to radians if needed
        double psi_rad = latest_cmd_traj_.psi;
        
        // Calculate direction vector from psi
        double dx = std::cos(psi_rad);
        double dy = std::sin(psi_rad);
        
        // Scale arrow length by velocity
        double arrow_length = latest_cmd_traj_.v * arrow_scale_ * 1.5;
        
        // Ensure arrow length is not zero
        if (arrow_length < 0.01) arrow_length = 0.01;
        
        // Create arrow points
        geometry_msgs::Point start, end;
        start.x = latest_cmd_traj_.x;
        start.y = latest_cmd_traj_.y;
        start.z = 0.2;  // Higher than trajectory line
        
        end.x = start.x + dx * arrow_length;
        end.y = start.y + dy * arrow_length;
        end.z = start.z;
        
        current_vel_marker.points.push_back(start);
        current_vel_marker.points.push_back(end);
        
        velocity_marker_pub_.publish(current_vel_marker);
    }
    
    // 添加清除轨迹的回调函数
    bool clearTrajectoryCallback(std_srvs::Empty::Request& request, 
                                std_srvs::Empty::Response& response) {
        ROS_INFO("收到清除轨迹请求");
        cmd_traj_points_.clear();
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_visualizer");
    
    TrajectoryVisualizer visualizer;
    
    ros::spin();
    
    return 0;
}
