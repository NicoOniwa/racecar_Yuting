#include <ros/ros.h>
#include <traj_analyzer/RefTraj.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <deque>
#include <cmath>

// Library for cubic spline interpolation
#include <Eigen/Dense>

class TrajectorySmoother {
private:
    ros::NodeHandle nh_;
    ros::Subscriber raw_traj_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Publisher smooth_traj_pub_;
    ros::Publisher nearest_point_pub_;
    
    std::deque<traj_analyzer::RefTraj> traj_buffer_;
    std::vector<traj_analyzer::RefTraj> interpolated_points_;
    geometry_msgs::PoseStamped current_pose_;
    bool pose_received_;
    
    int buffer_size_;
    int interpolation_points_;
    
public:
    TrajectorySmoother() {
        // Read parameters from ROS parameter server
        nh_.param<int>("buffer_size", buffer_size_, 20);
        nh_.param<int>("interpolation_points", interpolation_points_, 100);
        pose_received_ = false;
        
        // Publish smoothed trajectory (can be used for visualization)
        smooth_traj_pub_ = nh_.advertise<traj_analyzer::RefTraj>("smooth_trajectory", 10);
        
        // Publish nearest point
        nearest_point_pub_ = nh_.advertise<traj_analyzer::RefTraj>("nearest_trajectory_point", 10);

        // Subscribe raw trajectory
        raw_traj_sub_ = nh_.subscribe("raw_trajectory", 1000, &TrajectorySmoother::trajCallback, this);
        
        // Subscribe current pose
        current_pose_sub_ = nh_.subscribe("current_pose", 10, &TrajectorySmoother::poseCallback, this);
        
        // Timer for finding nearest point
        ros::Timer timer = nh_.createTimer(ros::Duration(0.05), &TrajectorySmoother::timerCallback, this);
    }
    
    void trajCallback(const traj_analyzer::RefTraj::ConstPtr& msg) {
        // Add new point to buffer
        traj_buffer_.push_back(*msg);
        
        // Keep buffer size
        if (traj_buffer_.size() > buffer_size_) {
            traj_buffer_.pop_front();
        }
        
        // Smooth trajectory when buffer size is reached
        if (traj_buffer_.size() == buffer_size_) {
            smoothTrajectory();
        }
    }
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        pose_received_ = true;
    }
    
    void timerCallback(const ros::TimerEvent&) {
        if (!pose_received_ || interpolated_points_.empty()) {
            return;
        }
        
        // Find nearest point
        int nearest_idx = findNearestPoint();
        
        // Publish nearest point
        nearest_point_pub_.publish(interpolated_points_[nearest_idx]);
    }
    
    int findNearestPoint() {
        if (interpolated_points_.empty()) {
            ROS_WARN("No interpolated points available");
            return 0;
        }
        
        double min_dist = std::numeric_limits<double>::max();
        int nearest_idx = 0;
        
        double current_x = current_pose_.pose.position.x;
        double current_y = current_pose_.pose.position.y;
        
        for (int i = 0; i < interpolated_points_.size(); i++) {
            double dx = interpolated_points_[i].x - current_x;
            double dy = interpolated_points_[i].y - current_y;
            double dist = dx * dx + dy * dy;  // Squared distance, no need for sqrt
            
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        return nearest_idx;
    }
    
    void smoothTrajectory() {
        // Extract points
        std::vector<double> x_points, y_points, psi_points, v_points, delta_points;
        
        for (const auto& point : traj_buffer_) {
            x_points.push_back(point.x);
            y_points.push_back(point.y);
            psi_points.push_back(point.psi);
            v_points.push_back(point.v);
            delta_points.push_back(point.delta);
        }
        
        // Create parameter vector (use arc length parameterization for better fitting)
        std::vector<double> params = computeArcLengthParams(x_points, y_points);
        
        // Perform cubic spline interpolation
        std::vector<double> x_interp, y_interp, psi_interp, v_interp, delta_interp, curvature_interp;
        cubicSplineInterpolation(params, x_points, y_points, psi_points, v_points, delta_points,
                                x_interp, y_interp, psi_interp, v_interp, delta_interp, curvature_interp);
        
        // Store interpolated points
        interpolated_points_.clear();
        
        for (int i = 0; i < interpolation_points_; i++) {
            traj_analyzer::RefTraj smooth_point;
            smooth_point.header.stamp = ros::Time::now();
            smooth_point.header.frame_id = "odom_capture_frame";
            smooth_point.x = x_interp[i];
            smooth_point.y = y_interp[i];
            smooth_point.psi = psi_interp[i];
            smooth_point.v = v_interp[i];
            smooth_point.delta = delta_interp[i];
            smooth_point.curvature = curvature_interp[i];
            
            interpolated_points_.push_back(smooth_point);
            
            // Only publish a few points for visualization
            if (i % 10 == 0) {
                smooth_traj_pub_.publish(smooth_point);
            }
        }
    }
    
    // Compute arc length parameterization for better spline fitting
    std::vector<double> computeArcLengthParams(const std::vector<double>& x_points, 
                                                  const std::vector<double>& y_points) {
        std::vector<double> params(x_points.size());
        params[0] = 0.0;
        
        for (size_t i = 1; i < x_points.size(); i++) {
            double dx = x_points[i] - x_points[i-1];
            double dy = y_points[i] - y_points[i-1];
            double segment_length = std::sqrt(dx*dx + dy*dy);
            params[i] = params[i-1] + segment_length;
        }
        
        return params;
    }
    
    // Optimize cubic spline interpolation
    void cubicSplineInterpolation(const std::vector<double>& params,
                                     const std::vector<double>& x_points,
                                     const std::vector<double>& y_points,
                                     const std::vector<double>& psi_points,
                                     const std::vector<double>& v_points,
                                     const std::vector<double>& delta_points,
                                     std::vector<double>& x_interp,
                                     std::vector<double>& y_interp,
                                     std::vector<double>& psi_interp,
                                     std::vector<double>& v_interp,
                                     std::vector<double>& delta_interp,
                                     std::vector<double>& curvature_interp) {
        // Pre-allocate memory
        x_interp.resize(interpolation_points_);
        y_interp.resize(interpolation_points_);
        psi_interp.resize(interpolation_points_);
        v_interp.resize(interpolation_points_);
        delta_interp.resize(interpolation_points_);
        curvature_interp.resize(interpolation_points_);
        
        // Use Eigen for cubic spline fitting - consider using a more efficient library
        // like Boost.Math or a custom implementation for better performance
        
        // Cache spline coefficients for reuse
        static Eigen::VectorXd x_coeff, y_coeff;
        static std::vector<double> last_params;
        
        bool recompute_coeffs = true;
        if (last_params.size() == params.size()) {
            recompute_coeffs = false;
            for (size_t i = 0; i < params.size(); i++) {
                if (std::abs(params[i] - last_params[i]) > 1e-6) {
                    recompute_coeffs = true;
                    break;
                }
            }
        }
        
        if (recompute_coeffs) {
            Eigen::MatrixXd A = buildSplineMatrix(params);
            x_coeff = solveSpline(A, x_points);
            y_coeff = solveSpline(A, y_points);
            last_params = params;
        }
        
        // Generate interpolated points in parallel if possible
        double t_step = (params.back() - params.front()) / (interpolation_points_ - 1);
        
        // Pre-compute interval indices to avoid repeated searches
        std::vector<int> interval_indices(interpolation_points_);
        for (int i = 0; i < interpolation_points_; i++) {
            double t = params.front() + i * t_step;
            int idx = 0;
            while (idx < params.size() - 1 && params[idx + 1] < t) {
                idx++;
            }
            interval_indices[i] = idx;
        }
        
        // Compute interpolated values
        for (int i = 0; i < interpolation_points_; i++) {
            double t = params.front() + i * t_step;
            int idx = interval_indices[i];
            
            // Calculate interpolated points
            x_interp[i] = evaluateSpline(x_coeff, params, t, idx);
            y_interp[i] = evaluateSpline(y_coeff, params, t, idx);
            
            // Calculate first derivative
            double dx_dt = evaluateSplineDerivative(x_coeff, params, t, idx, 1);
            double dy_dt = evaluateSplineDerivative(y_coeff, params, t, idx, 1);
            
            // Calculate second derivative
            double d2x_dt2 = evaluateSplineDerivative(x_coeff, params, t, idx, 2);
            double d2y_dt2 = evaluateSplineDerivative(y_coeff, params, t, idx, 2);
            
            // Calculate curvature
            double denominator = std::pow(dx_dt * dx_dt + dy_dt * dy_dt, 1.5);
            if (denominator > 1e-10) {  // Avoid division by near-zero
                curvature_interp[i] = (dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / denominator;
            } else {
                curvature_interp[i] = 0.0;
            }
            
            // Simple linear interpolation for other parameters
            psi_interp[i] = linearInterpolate(params, psi_points, t);
            v_interp[i] = linearInterpolate(params, v_points, t);
            delta_interp[i] = linearInterpolate(params, delta_points, t);
        }
    }
    
    Eigen::MatrixXd buildSplineMatrix(const std::vector<double>& params) {
        int n = params.size();
        
        // For n points, we have n-1 segments, each with 4 coefficients (a, b, c, d)
        // So we need to solve for 4(n-1) unknowns
        int num_unknowns = 4 * (n - 1);
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_unknowns, num_unknowns);
        
        // We'll set up the constraints for the cubic spline:
        // 1. Spline passes through each data point (2n equations)
        // 2. First derivatives are continuous at interior points (n-2 equations)
        // 3. Second derivatives are continuous at interior points (n-2 equations)
        // 4. Second derivatives at endpoints are zero (natural spline) (2 equations)
        
        // Constraint 1: Spline passes through each data point
        // S_i(t_i) = y_i and S_i(t_{i+1}) = y_{i+1}
        for (int i = 0; i < n - 1; i++) {
            // S_i(t_i) = a_i
            A(2*i, 4*i) = 1.0;  // a_i coefficient
            
            // S_i(t_{i+1}) = a_i + b_i*h_i + c_i*h_i^2 + d_i*h_i^3
            double h_i = params[i+1] - params[i];
            A(2*i+1, 4*i) = 1.0;                  // a_i
            A(2*i+1, 4*i+1) = h_i;                // b_i
            A(2*i+1, 4*i+2) = h_i * h_i;          // c_i
            A(2*i+1, 4*i+3) = h_i * h_i * h_i;    // d_i
        }
        
        // Constraint 2: First derivatives are continuous at interior points
        // S'_i(t_{i+1}) = S'_{i+1}(t_{i+1}) for i = 0,...,n-3
        for (int i = 0; i < n - 2; i++) {
            double h_i = params[i+1] - params[i];
            // S'_i(t_{i+1}) = b_i + 2*c_i*h_i + 3*d_i*h_i^2
            A(2*(n-1)+i, 4*i+1) = 1.0;                // b_i
            A(2*(n-1)+i, 4*i+2) = 2.0 * h_i;          // c_i
            A(2*(n-1)+i, 4*i+3) = 3.0 * h_i * h_i;    // d_i
            
            // S'_{i+1}(t_{i+1}) = b_{i+1}
            A(2*(n-1)+i, 4*(i+1)+1) = -1.0;           // -b_{i+1}
        }
        
        // Constraint 3: Second derivatives are continuous at interior points
        // S''_i(t_{i+1}) = S''_{i+1}(t_{i+1}) for i = 0,...,n-3
        for (int i = 0; i < n - 2; i++) {
            double h_i = params[i+1] - params[i];
            // S''_i(t_{i+1}) = 2*c_i + 6*d_i*h_i
            A(2*(n-1)+(n-2)+i, 4*i+2) = 2.0;          // c_i
            A(2*(n-1)+(n-2)+i, 4*i+3) = 6.0 * h_i;    // d_i
            
            // S''_{i+1}(t_{i+1}) = 2*c_{i+1}
            A(2*(n-1)+(n-2)+i, 4*(i+1)+2) = -2.0;     // -c_{i+1}
        }
        
        // Constraint 4: Natural spline boundary conditions (second derivatives at endpoints are zero)
        // S''_0(t_0) = 0 and S''_{n-2}(t_{n-1}) = 0
        
        // S''_0(t_0) = 2*c_0 = 0
        A(num_unknowns-2, 2) = 2.0;  // c_0
        
        // S''_{n-2}(t_{n-1}) = 2*c_{n-2} + 6*d_{n-2}*h_{n-2} = 0
        double h_last = params[n-1] - params[n-2];
        A(num_unknowns-1, 4*(n-2)+2) = 2.0;           // c_{n-2}
        A(num_unknowns-1, 4*(n-2)+3) = 6.0 * h_last;  // d_{n-2}
        
        return A;
    }
    
    Eigen::VectorXd solveSpline(const Eigen::MatrixXd& A, const std::vector<double>& points) {
        int n = points.size();
        int num_unknowns = 4 * (n - 1);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(num_unknowns);
        
        // Set up the right-hand side of the equation Ax = b
        
        // Constraint 1: Spline passes through each data point
        // S_i(t_i) = y_i and S_i(t_{i+1}) = y_{i+1}
        for (int i = 0; i < n - 1; i++) {
            // S_i(t_i) = y_i
            b(2*i) = points[i];
            
            // S_i(t_{i+1}) = y_{i+1}
            b(2*i+1) = points[i+1];
        }
        
        // Constraint 2: First derivatives are continuous at interior points
        // All zeros on the right-hand side (already initialized)
        
        // Constraint 3: Second derivatives are continuous at interior points
        // All zeros on the right-hand side (already initialized)
        
        // Constraint 4: Natural spline boundary conditions (second derivatives at endpoints are zero)
        // All zeros on the right-hand side (already initialized)
        
        // Solve the linear system
        Eigen::VectorXd coeff;
        
        // Check if matrix is invertible
        Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
        if (lu.isInvertible()) {
            // Use a more stable solver for potentially ill-conditioned matrices
            coeff = A.colPivHouseholderQr().solve(b);
            
            // Check solution quality
            double relative_error = (A * coeff - b).norm() / b.norm();
            if (relative_error > 1e-6) {
                ROS_WARN_STREAM("Spline solution has high relative error: " << relative_error);
            }
        } else {
            // Matrix is not invertible, use a fallback approach
            ROS_WARN("Spline matrix is not invertible, using SVD solver");
            coeff = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        }
        
        return coeff;
    }
    
    double evaluateSpline(const Eigen::VectorXd& coeff, const std::vector<double>& params, double t, int idx) {
        double dt = t - params[idx];
        int offset = idx * 4;
        return coeff(offset) + coeff(offset + 1) * dt + coeff(offset + 2) * dt * dt + coeff(offset + 3) * dt * dt * dt;
    }
    
    double evaluateSplineDerivative(const Eigen::VectorXd& coeff, const std::vector<double>& params, double t, int idx, int order) {
        double dt = t - params[idx];
        int offset = idx * 4;
        
        if (order == 1) {
            return coeff(offset + 1) + 2 * coeff(offset + 2) * dt + 3 * coeff(offset + 3) * dt * dt;
        } else if (order == 2) {
            return 2 * coeff(offset + 2) + 6 * coeff(offset + 3) * dt;
        } else {
            return 0.0;
        }
    }
    
    double linearInterpolate(const std::vector<double>& x, const std::vector<double>& y, double x_val) {
        // Check if vectors have the same size
        if (x.size() != y.size() || x.empty()) {
            ROS_WARN("Invalid input for linear interpolation");
            return 0.0;
        }
        
        // Check if x_val is out of bounds
        if (x_val <= x.front()) return y.front();
        if (x_val >= x.back()) return y.back();
        
        // Find corresponding interval
        int idx = 0;
        while (idx < x.size() - 1 && x[idx + 1] < x_val) {
            idx++;
        }
        
        // Linear interpolation
        double t = (x_val - x[idx]) / (x[idx + 1] - x[idx]);
        return y[idx] + t * (y[idx + 1] - y[idx]);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_smoother");
    
    TrajectorySmoother smoother;
    
    ros::spin();
    
    return 0;
}