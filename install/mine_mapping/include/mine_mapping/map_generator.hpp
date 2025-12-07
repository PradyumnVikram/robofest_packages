#ifndef MAP_GENERATOR_HPP
#define MAP_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

struct MineLocation {
    double x, y;
};

class MapGen : public rclcpp::Node {  // ← Inherit from Node
public:
    MapGen();
    std::vector<MineLocation> mine_locations;
    void update_map(const cv::Mat& frame, double* curr_pos);

private:
    bool mine_exists(const MineLocation& mine) const {
        const double error = 5;
        for (const auto& m : mine_locations) {
            if (std::hypot(m.x - mine.x, m.y - mine.y) < error) {
                return true;
            }
        }
        return false;
    }
};

#endif
