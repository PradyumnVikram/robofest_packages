#ifndef MAP_GENERATOR_HPP
#define MAP_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <Eigen/Geometry>

struct MineLocation {
    double x, y;
};

struct DronePose {
    double x, y, z;           // NED position (z negative = altitude)
    double q_w, q_x, q_y, q_z; // Quaternion orientation [w,x,y,z]
    double altitude() const { return -z; }  // Positive altitude
};

class MapGen : public rclcpp::Node {  // ← Inherit from Node
public:
    MapGen();
    std::vector<MineLocation> mine_locations;
    void update_map(const cv::Mat& frame, DronePose& drone_pose);
    MineLocation pixel_to_world(double pixel_x, double pixel_y, DronePose& drone_pose);

private:
    bool mine_exists(const MineLocation& mine) const {
        const double error = 20;
        for (const auto& m : mine_locations) {
            if (std::hypot(m.x - mine.x, m.y - mine.y) < error) {
                return true;
            }
        }
        return false;
    }
};

#endif
