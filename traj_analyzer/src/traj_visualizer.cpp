#include <ros/ros.h>
#include <traj_analyzer/RefTraj.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

class TrajectoryVisualizer {
private:
    ros::NodeHandle nh_;
    ros::Subscriber raw_traj_sub_;
    ros::Publisher raw_marker_pub_;
    
    std::vector<traj_analyzer::RefTraj> raw_points_;
    
    std::string frame_id_;
    ros::Timer timer_;
    double point_size_;
    double line_width_;
    
public:
    TrajectoryVisualizer() {
        // Get parameters
        nh_.param<std::string>("frame_id", frame_id_, "odom_capture_frame");
        nh_.param<double>("point_size", point_size_, 0.2);
        nh_.param<double>("line_width", line_width_, 0.05);
        
        // Publishers for visualization markers
        raw_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("raw_trajectory_markers", 1);
        
        // Subscribers
        raw_traj_sub_ = nh_.subscribe("raw_trajectory", 100, &TrajectoryVisualizer::rawTrajCallback, this);
        
        // Timer for visualization update
        timer_ = nh_.createTimer(ros::Duration(0.1), &TrajectoryVisualizer::timerCallback, this);
    }
    
    void rawTrajCallback(const traj_analyzer::RefTraj::ConstPtr& msg) {
        // Store raw trajectory points (with a maximum buffer size)
        raw_points_.push_back(*msg);
        if (raw_points_.size() > 100) {
            raw_points_.erase(raw_points_.begin());
        }
    }
    
    void timerCallback(const ros::TimerEvent&) {
        // Publish visualization markers
        publishRawTrajectoryMarker();
    }
    
    void publishRawTrajectoryMarker() {
        if (raw_points_.empty()) return;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "raw_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = line_width_;  // line width
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.5);  // short lifetime to auto-clear old points
        
        for (const auto& point : raw_points_) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.05;  // slightly above ground
            marker.points.push_back(p);
        }
        
        raw_marker_pub_.publish(marker);
        
        // Also publish points as spheres
        visualization_msgs::Marker point_marker = marker;
        point_marker.id = 1;
        point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        point_marker.scale.x = point_marker.scale.y = point_marker.scale.z = point_size_;
        raw_marker_pub_.publish(point_marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_visualizer");
    
    TrajectoryVisualizer visualizer;
    
    ros::spin();
    
    return 0;
}