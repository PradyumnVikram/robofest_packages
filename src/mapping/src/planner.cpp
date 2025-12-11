#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"  // 👉 ADDED for drone pose

// GCOPTER HEADERS
#include "firi.hpp"
#include "flatness.hpp"
#include "gcopter.hpp"
#include "geo_utils.hpp"
#include "lbfgs.hpp"
#include "minco.hpp"
#include "quickhull.hpp"
#include "root_finder.hpp"
#include "sdlp.hpp"
#include "sfc_gen.hpp"
#include "trajectory.hpp"

using namespace std;
using namespace Eigen;

class GCOPTERTrajectoryPlanner : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr h_matrix_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex h_matrix_mutex;
    Eigen::MatrixXd current_H_3d;
    Eigen::VectorXd current_h_3d;
    std::atomic<bool> ready_to_process{false};
    Eigen::Vector3d current_drone_pos{0.0, 0.0, 1.0};  // 👉 LIVE drone tracking

public:
    GCOPTERTrajectoryPlanner() : Node("gcopter_trajectory_planner") {
        h_matrix_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/corridor_h_matrix", 10, std::bind(&GCOPTERTrajectoryPlanner::hMatrixCallback, this, std::placeholders::_1));
        
        // 👉 LIVE DRONE POSITION TRACKING
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                    "/fmu/out/vehicle_odometry",   // PX4 odom topic
                    10,
                    [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                        // position is [x, y, z] in meters, local frame
                        current_drone_pos = Eigen::Vector3d(
                            msg->position[0],
                            msg->position[1],
                            msg->position[2]);

                        RCLCPP_INFO_THROTTLE(
                            get_logger(), *get_clock(), 2000,
                            "🛰️ LIVE Drone (odom): [%.2f, %.2f, %.2f]",
                            current_drone_pos(0),
                            current_drone_pos(1),
                            current_drone_pos(2));
                    });

        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&GCOPTERTrajectoryPlanner::processTimerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "🚀 LIVE DRONE TRACKING + REAL H-REPS GCOPTER!");
    }

private:
    void hMatrixCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if(msg->data.size() < 4) return;
        
        size_t num_constraints = static_cast<size_t>(msg->data[0]);
        if(num_constraints > 12) num_constraints = 12;
        
        {
            std::lock_guard<std::mutex> lock(h_matrix_mutex);
            current_H_3d.resize(num_constraints, 4);
            current_h_3d.resize(num_constraints);
            
            size_t offset = 4;
            for(size_t i = 0; i < num_constraints; i++) {
                current_H_3d(i, 0) = msg->data[offset + i*2 + 0];
                current_H_3d(i, 1) = msg->data[offset + i*2 + 1];
                current_H_3d(i, 2) = 0.0;
                current_H_3d(i, 3) = 0.0;
                current_h_3d(i) = msg->data[offset + num_constraints*2 + i];
            }
        }
        
        ready_to_process = true;
    }

    void processTimerCallback() {
        if(!ready_to_process) return;
        
        std::lock_guard<std::mutex> lock(h_matrix_mutex);
        size_t num_real = std::min(static_cast<size_t>(current_H_3d.rows()), size_t(8));
        
        // 👉 FILTER: H-reps that CONTAIN current drone position
        std::vector<Eigen::Vector4d> valid_hreps;
        for(size_t i = 0; i < num_real; i++) {
            double norm_xy = sqrt(current_H_3d(i,0)*current_H_3d(i,0) + current_H_3d(i,1)*current_H_3d(i,1));
            if(norm_xy > 1e-3 && abs(current_h_3d(i)) > 0.05) {
                Eigen::Vector4d hrep;
                hrep << current_H_3d(i,0)/norm_xy, current_H_3d(i,1)/norm_xy, 0.0, current_h_3d(i)/norm_xy;
                
                // 👉 ONLY keep H-reps where drone CURRENTLY satisfies constraint
                if(hrep.head<3>().dot(current_drone_pos) <= hrep(3) + 0.2) {
                    valid_hreps.push_back(hrep);
                }
            }
        }
        
        size_t num_valid = valid_hreps.size();
        RCLCPP_INFO(get_logger(), "✅ %zu/%zu lidar H-reps contain drone [%.2f,%.2f,%.2f]", 
                   num_valid, num_real, current_drone_pos(0), current_drone_pos(1), current_drone_pos(2));
        
        if(num_valid < 3) {
            RCLCPP_WARN(get_logger(), "⚠️ Too few H-reps (%zu) - need 3+", num_valid);
            ready_to_process = false;
            return;
        }
        
        // 👉 BUILD corridor from VALID drone-containing H-reps
        Eigen::MatrixX4d corridor(num_valid + 6, 4);
        for(size_t i = 0; i < num_valid; i++) {
            corridor.row(i) = valid_hreps[i];
        }
        
        // Drone-sized safety bounds around current position
        double drone_x = current_drone_pos(0), drone_y = current_drone_pos(1), drone_z = current_drone_pos(2);
        corridor.row(num_valid + 0) << 0,0,1, drone_z+2.0;     // z ceiling
        corridor.row(num_valid + 1) << 0,0,-1, -(drone_z-0.5); // z floor
        corridor.row(num_valid + 2) << 1,0,0, drone_x+4.0;     // x forward
        corridor.row(num_valid + 3) << -1,0,0, -(drone_x+3.0); // x backward
        corridor.row(num_valid + 4) << 0,1,0, drone_y+3.0;     // y right
        corridor.row(num_valid + 5) << 0,-1,0, -(drone_y+3.0); // y left
        
        std::vector<Eigen::MatrixX4d> safeCorridor = {corridor};
        
        // 👉 GOAL: 1.5m forward from drone (reachable!)
        Eigen::Vector3d goal_pos = current_drone_pos + Eigen::Vector3d(1.5, 0.0, 0.0);
        
        Eigen::Matrix3d headPVA, tailPVA; headPVA.setZero(); tailPVA.setZero();
        headPVA.col(0) = current_drone_pos;  // CURRENT position
        tailPVA.col(0) = goal_pos;           // REACHABLE goal
        
        // Drone PX4 parameters
        Eigen::VectorXd bounds(6), weights(6), params(6);
        bounds << 2.5, 8.0, 1.0, -1.2, 1.5, 4.0;  // v_max, ω_max, θ_max, T_min, T_max
        weights << 1.5,1.5,1.0,0.05,0.05,1.0;     // Smooth trajectory
        params << 0.5, 9.81, 0,0,0,1.0;           // Drone mass, gravity
        
        gcopter::GCOPTER_PolytopeSFC sfc;
        bool setup_ok = sfc.setup(0.7, headPVA, tailPVA, safeCorridor,
                                 0.25, 0.08, 4, bounds, weights, params);
        
        RCLCPP_INFO(get_logger(), "🔍 REAL setup: %s | %zu lidar walls | Goal[%.1f,%.1f,%.1f]", 
                   setup_ok ? "✅ OK" : "❌ FAIL", num_valid, 
                   goal_pos(0), goal_pos(1), goal_pos(2));
        
        if(setup_ok) {
            Trajectory<3> traj;
            double cost = sfc.optimize(traj, 3e-3);
            
            if(traj.getPieceNum() > 0) {
                saveTrajectoryToCSV(traj, "/home/azidozide/live_drone_trajectory.csv");
                RCLCPP_INFO(get_logger(), "🎉 SUCCESS! %.1fs | %d pieces | Cost: %.3f 🚀", 
                           traj.getTotalDuration(), traj.getPieceNum(), cost);
            } else {
                RCLCPP_ERROR(get_logger(), "💥 Optimization FAILED - corridor unreachable");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "💥 Setup FAILED - drone trapped by %zu lidar walls!", num_valid);
        }
        
        ready_to_process = false;
    }

    void saveTrajectoryToCSV(const Trajectory<3>& traj, const std::string& filename) {
        std::ofstream fout(filename);
        if(!fout.is_open()) {
            RCLCPP_ERROR(get_logger(), "💥 Cannot write %s", filename.c_str());
            return;
        }
        
        double t = 0.0, dt = 0.001;
        int pieceNum = traj.getPieceNum();
        
        fout << "time,x,y,z,vx,vy,vz,ax,ay,az\n";  // PX4 header
        fout << fixed << setprecision(6);
        
        for(int seg = 0; seg < pieceNum && t < 15.0; seg++) {
            auto cMat = traj[seg].getCoeffMat();
            double duration = traj[seg].getDuration();
            
            while(t < duration + 0.001) {
                double t_rel = t;
                double pos[3], vel[3], acc[3];
                
                for(int i = 0; i < 3; i++) {
                    pos[i] = cMat(i,3) + t_rel*cMat(i,2) + t_rel*t_rel*cMat(i,1) + t_rel*t_rel*t_rel*cMat(i,0);
                    vel[i] = cMat(i,2) + 2*t_rel*cMat(i,1) + 3*t_rel*t_rel*cMat(i,0);
                    acc[i] = 2*cMat(i,1) + 6*t_rel*cMat(i,0);
                }
                
                fout << t << "," << pos[0] << "," << pos[1] << "," << pos[2] << ","
                     << vel[0] << "," << vel[1] << "," << vel[2] << ","
                     << acc[0] << "," << acc[1] << "," << acc[2] << "\n";
                t += dt;
            }
        }
        fout.close();
        RCLCPP_INFO(get_logger(), "💾 PX4 trajectory: %s", filename.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<GCOPTERTrajectoryPlanner>();
    rclcpp::spin(planner);
    rclcpp::shutdown();
    return 0;
}
