#include "map_generator.hpp"

MapGen::MapGen() : rclcpp::Node("map_generator") {  // ← Proper Node constructor
    mine_locations.clear();
    RCLCPP_INFO(this->get_logger(), "MapGen initialized");
}

void MapGen::update_map(const cv::Mat& frame, double* curr_pos) {
    RCLCPP_INFO(this->get_logger(), "=== Frame processing started ===");
    
    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty frame received - skipping");
        return;
    }
    
    cv::Mat display_frame = frame.clone();  // For annotations
    RCLCPP_INFO(this->get_logger(), "Frame size: %dx%d", frame.cols, frame.rows);

    // Step 1: Grayscale
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::imshow("1_Gray", gray);
    RCLCPP_DEBUG(this->get_logger(), "✓ Grayscale conversion complete");

    // Step 2: Thresholding
    cv::Mat mask;
    cv::threshold(gray, mask, 75, 255, cv::THRESH_BINARY_INV);
    cv::imshow("2_Mask", mask);
    RCLCPP_INFO(this->get_logger(), "Mask pixels: %d", cv::countNonZero(mask));

    // Step 3: Morphology
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::imshow("3_Morphology", mask);

    // Step 4: Contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_INFO(this->get_logger(), "Found %zu contours", contours.size());

    // Step 5: Annotate EVERYTHING
    int valid_mines = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        const auto& contour = contours[i];
        double area = cv::contourArea(contour);
        
        // Draw ALL contours (GREEN thin)
        cv::drawContours(display_frame, contours, i, cv::Scalar(0, 255, 0), 1);
        
        if (area > 100) {
            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0) {
                double pixel_x = m.m10 / m.m00;
                double pixel_y = m.m01 / m.m00;
                
                // Draw center (YELLOW circle)
                cv::circle(display_frame, cv::Point(pixel_x, pixel_y), 6, cv::Scalar(0, 255, 255), 2);
                
                // Area label
                cv::putText(display_frame, std::to_string((int)area), 
                           cv::Point(pixel_x, pixel_y-15), cv::FONT_HERSHEY_SIMPLEX, 
                           0.5, cv::Scalar(0, 255, 255), 2);
                
                double mine_x = pixel_x - frame.cols / 2.0;
                double mine_y = pixel_y - frame.rows / 2.0;
                mine_x -= curr_pos[0];
                mine_y -= curr_pos[1];
                
                MineLocation new_mine = {mine_x, mine_y};
                if (!mine_exists(new_mine)) {
                    mine_locations.push_back(new_mine);
                    
                    // **MINE** label (CYAN thick)
                    cv::putText(display_frame, "MINE", 
                               cv::Point(pixel_x-20, pixel_y+25), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 3);
                    
                    // Global coords
                    char coords[50];
                    sprintf(coords, "%.1fm %.1fm", mine_x, mine_y);
                    cv::putText(display_frame, coords, 
                               cv::Point(pixel_x, pixel_y+45), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
                    
                    RCLCPP_INFO(this->get_logger(), "✓ MINE ADDED: %.2f, %.2f (area=%.1f)", 
                               mine_x, mine_y, area);
                    valid_mines++;
                } else {
                    // Duplicate (RED X)
                    cv::line(display_frame, cv::Point(pixel_x-10, pixel_y-10), 
                            cv::Point(pixel_x+10, pixel_y+10), cv::Scalar(0, 0, 255), 2);
                    cv::line(display_frame, cv::Point(pixel_x+10, pixel_y-10), 
                            cv::Point(pixel_x-10, pixel_y+10), cv::Scalar(0, 0, 255), 2);
                    RCLCPP_DEBUG(this->get_logger(), "Duplicate mine");
                }
            }
        }
    }
    
    // CENTER CROSSHAIR (BLUE)
    int cx = frame.cols / 2, cy = frame.rows / 2;
    cv::line(display_frame, cv::Point(cx-30, cy), cv::Point(cx+30, cy), cv::Scalar(255, 0, 0), 3);
    cv::line(display_frame, cv::Point(cx, cy-30), cv::Point(cx, cy+30), cv::Scalar(255, 0, 0), 3);
    cv::circle(display_frame, cv::Point(cx, cy), 5, cv::Scalar(255, 0, 0), -1);
    
    // DRONE POSITION TEXT
    char drone_pos[50];
    sprintf(drone_pos, "Drone: %.1f, %.1f", curr_pos[0], curr_pos[1]);
    cv::putText(display_frame, drone_pos, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    
    cv::imshow("4_Annotated", display_frame);
    cv::waitKey(1);
    
    RCLCPP_INFO(this->get_logger(), "=== Complete: %d new mines, total=%zu ===", 
                valid_mines, mine_locations.size());
}
