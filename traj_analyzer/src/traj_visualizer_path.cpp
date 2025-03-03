#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <traj_analyzer/RefTraj.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TrajectoryVisualizer {
private:
    ros::NodeHandle nh_;
    ros::Subscriber raw_traj_sub_;
    ros::Publisher path_pub_;  // Path发布器
    ros::Publisher vel_markers_pub_;  // 新增速度/转向可视化发布器
    
    std::vector<traj_analyzer::RefTraj> raw_points_;
    std::string frame_id_;     // 坐标系
    int max_points_;        // 最大轨迹点数
    // 可视化参数
    double max_velocity_;
    double arrow_scale_;

    ros::Timer timer_;
    
public:
    TrajectoryVisualizer() {
        nh_.param<std::string>("frame_id", frame_id_, "odom");
        nh_.param<int>("max_points", max_points_, 50);
        nh_.param<double>("max_velocity", max_velocity_, 5.0);  // 用于颜色归一化
        nh_.param<double>("arrow_scale", arrow_scale_, 0.5);   // 箭头长度缩放因子
        
        path_pub_ = nh_.advertise<nav_msgs::Path>("trajectory_path", 1);
        vel_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("velocity_markers", 1);
        raw_traj_sub_ = nh_.subscribe("raw_trajectory", 100, &TrajectoryVisualizer::rawTrajCallback, this);
        
        // 定时器保证可视化刷新（0.1秒刷新率）
        timer_ = nh_.createTimer(ros::Duration(0.1), 
                                          &TrajectoryVisualizer::timerCallback, this);
    }
    
    void rawTrajCallback(const traj_analyzer::RefTraj::ConstPtr& msg) {
        raw_points_.push_back(*msg);
        
        // 保持指定缓存长度
        if(raw_points_.size() > max_points_) {
            raw_points_.erase(raw_points_.begin());
        }
    }
    
    void timerCallback(const ros::TimerEvent&) {
        if(!raw_points_.empty()){
            publishRefPath();
            publishVelocityMarkers();
        }
    }
    
    void publishRefPath() {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();  // 使用当前时间为Header
        path.header.frame_id = frame_id_;
        
        for(const auto& point : raw_points_){
            geometry_msgs::PoseStamped pose;
            
            // 继承轨迹点的原始时间戳（如有需要可改用点自身的时间戳）
            pose.header.stamp = path.header.stamp;  
            pose.header.frame_id = frame_id_;
            
            // 设置位置
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.1;  // 稍微高出地面以便观察
            
            // 将航向角转换为四元数
            tf2::Quaternion q;
            q.setRPY(0, 0, point.psi);  // roll=0，pitch=0，yaw=psi
            pose.pose.orientation = tf2::toMsg(q);
            
            path.poses.push_back(pose);
        }
        
        path_pub_.publish(path);
    }

    void publishVelocityMarkers() {
        visualization_msgs::MarkerArray marker_array;

        for(size_t i = 0; i < raw_points_.size(); ++i) {
            const auto& point = raw_points_[i];
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "velocity_direction";
            marker.id = i;  // 唯一ID
            marker.action = visualization_msgs::Marker::ADD;

            // 基本属性设置
            marker.pose.orientation.w = 1.0;  // 方向由points决定的情况不适用
            marker.color = getColorByVelocity(point.v);

            // 计算箭头方向和长度
            double direction = point.psi + point.delta;  // 偏航角 + 舵角方向
            double arrow_length = point.v * arrow_scale_;

            // 处理速度为0的情况
            if(point.v < 0.01) {
                createPointMarker(marker, point);
            } else {
                createArrowMarker(marker, point, direction, arrow_length);
            }

            marker_array.markers.push_back(marker);
        }

        // 清理旧标记（重要！）
        visualization_msgs::Marker clean_marker;
        clean_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clean_marker);

        vel_markers_pub_.publish(marker_array);
    }


    std_msgs::ColorRGBA getColorByVelocity(double v) {
        std_msgs::ColorRGBA color;
        color.a = 0.8;  // 透明度
        
        // 颜色渐变：红 -> 绿
        double ratio = std::min(std::abs(v) / max_velocity_, 1.0);
        color.r = ratio;
        color.g = 1.0 - ratio;
        color.b = 0.2;
        return color;
    }

    void createArrowMarker(visualization_msgs::Marker& marker,
                           const traj_analyzer::RefTraj& point,
                           double direction,
                           double length) {
        marker.type = visualization_msgs::Marker::ARROW;
        
        // 设置位置和方向
        geometry_msgs::Point start;
        start.x = point.x;
        start.y = point.y;
        start.z = 0.2;  // 略高于轨迹线
        
        geometry_msgs::Point end;
        end.x = start.x + length * cos(direction);
        end.y = start.y + length * sin(direction);
        end.z = 0.2;

        marker.points.push_back(start);
        marker.points.push_back(end);

        // 设置箭头尺寸
        marker.scale.x = 0.05;  // 箭头杆粗细
        marker.scale.y = 0.1;   // 箭头头宽度
        marker.scale.z = 0.15;  // 箭头头高度
    }

    void createPointMarker(visualization_msgs::Marker& marker,
                          const traj_analyzer::RefTraj& point) {
        marker.type = visualization_msgs::Marker::SPHERE;
        
        // 设置位置
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = 0.2;

        // 设置点尺寸和颜色（保持与箭头相同的颜色）
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_visualizer_cpp");
    
    TrajectoryVisualizer visualizer;
    ros::spin();
    
    return 0;
}
