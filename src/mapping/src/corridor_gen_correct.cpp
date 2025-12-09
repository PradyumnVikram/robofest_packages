#include <iostream>
#include <Eigen/Dense>
#include <limits>
#include <string>
#include <fstream>
#include <math.h>
#include <cmath>
#include <vector>
#include <cstring>
#include <iomanip>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

// ROS2 includes (add these)
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <functional>   // for std::bind
#include "quickhull2.hpp"
#include "linear_eq.hpp"
#include "final_hstar.hpp"
#include <chrono>
#include <thread>
#include <mutex>
using namespace quickhull;
using namespace line_eq;


using namespace std;
using namespace Eigen;

class polyhedronGenerator {
public:
    polyhedronGenerator(int max_x_id, int max_y_id) {
        _max_x_id = max_x_id;
        _max_y_id = max_y_id;
        _grid_num = max_x_id * max_y_id;
        
        map_data = MatrixXi::Zero(_max_x_id, _max_y_id);
        use_data = MatrixXi::Zero(_max_x_id, _max_y_id);
        invalid_data = MatrixXi::Zero(_max_x_id, _max_y_id);
        inside_data = MatrixXi::Zero(_max_x_id, _max_y_id);
        
        vertex_idx = VectorXi::Zero(8);
        vertex_idx_lst = VectorXi::Zero(8);
        
        inf_step = 1;
        itr_inflate_max = 1000;
        _cluster_buffer_size = 50000;
        _candidate_buffer_size = 10000;
    }
    
    ~polyhedronGenerator() {}

    void mapClear() {
        map_data.setZero();
    }

    void flagClear() {
        use_data.setZero();
        invalid_data.setZero();
        inside_data.setZero();
    }

    void paramSet(bool is_gpu_on_stage_1, bool is_gpu_on_stage_2, bool is_cluster_on,
                  const int & max_x_id, const int & max_y_id, double resolution, 
                  double itr_inflate_max_, double itr_cluster_max_) {
        _resolution = resolution;
        _max_x_id = max_x_id;
        _max_y_id = max_y_id;
        _grid_num = max_x_id * max_y_id;

        if(is_cluster_on) {
            itr_inflate_max = itr_inflate_max_;
            itr_cluster_max = itr_cluster_max_;
        }
        else {
            itr_inflate_max = 1000;
            itr_cluster_max = 0;   
        }

        _cluster_buffer_size = 50000;
        _candidate_buffer_size = 10000;
        _cluster_buffer_size_square = _candidate_buffer_size * _candidate_buffer_size;

        map_data = MatrixXi::Zero(max_x_id, max_y_id);
        use_data = MatrixXi::Zero(max_x_id, max_y_id);
        invalid_data = MatrixXi::Zero(max_x_id, max_y_id);
        inside_data = MatrixXi::Zero(max_x_id, max_y_id);

        active_xy_id = MatrixXi::Zero(_candidate_buffer_size, 2);
        cluster_xy_id = MatrixXi::Zero(_cluster_buffer_size, 2);
        candidate_cell = MatrixXi::Zero(_candidate_buffer_size, 2);

        inf_step = 1;
        mapClear();
    }

    void setObs(const int & id_x, const int & id_y) {
        if(id_x >= 0 && id_x < _max_x_id && id_y >= 0 && id_y < _max_y_id) {
            map_data(id_x, id_y) = 1;
        }
    }

    void setVertexInitIndex(VectorXi& vertex_idx, int min_x, int min_y, int max_x, int max_y) {
        vertex_idx(0) = vertex_idx(1) = max_x;  
        vertex_idx(2) = vertex_idx(3) = min_x;  
        vertex_idx(4) = vertex_idx(7) = min_y;
        vertex_idx(5) = vertex_idx(6) = max_y;
    }

    void getGridsInRect(const VectorXi& vertex_idx, 
                         vector<int>& cube_grid_x, 
                         vector<int>& cube_grid_y) {
        int id_x, id_y;
        int min_x = vertex_idx(2);
        int max_x = vertex_idx(0);
        int min_y = vertex_idx(4);
        int max_y = vertex_idx(5);
        
        for(id_x = min_x; id_x <= max_x; id_x++) {   
            for(id_y = min_y; id_y <= max_y; id_y++) {
                cube_grid_x.push_back(id_x);
                cube_grid_y.push_back(id_y);
                if(map_data(id_x,id_y)!=1) 
                    {inside_data(id_x, id_y) = 6;}
            }
        }
    }

    void inflateX_n(VectorXi& vertex_idx) {
        if(vertex_idx(0 + 3) == 0) return;     
        int id_x = vertex_idx(0 + 3) - 1;   
        for(int id_y = vertex_idx(4 + 3); id_y <= vertex_idx(4 + 2); id_y++) {
            if(map_data(id_x, id_y) == 1) return;
        }
        vertex_idx(2) -= inf_step;
        vertex_idx(3) -= inf_step;
    }

    void inflateX_p(VectorXi& vertex_idx) {
        if(vertex_idx(0 + 0) == _max_x_id - 1) return;     
        int id_x = vertex_idx(0 + 0) + 1;   
        for(int id_y = vertex_idx(4 + 0); id_y <= vertex_idx(4 + 1); id_y++) {
            if(map_data(id_x, id_y) ==1) return;
        }
        vertex_idx(0) += inf_step;
        vertex_idx(1) += inf_step;
    }

    void inflateY_n(VectorXi& vertex_idx) {
        if(vertex_idx(4 + 0) == 0) return;     
        int id_y = vertex_idx(4 + 0) - 1;   
        for(int id_x = vertex_idx(0 + 3); id_x <= vertex_idx(0 + 0); id_x++) {
            if(map_data(id_x, id_y)==1) return;
        }
        vertex_idx(4) -= inf_step;
        vertex_idx(7) -= inf_step;
    }

    void inflateY_p(VectorXi& vertex_idx) {
        if(vertex_idx(4 + 1) == _max_y_id - 1) return;     
        int id_y = vertex_idx(4 + 1) + 1;   
        for(int id_x = vertex_idx(0 + 2); id_x <= vertex_idx(0 + 1); id_x++) {
            if(map_data(id_x, id_y) ==1) return;
        }
        vertex_idx(5) += inf_step;
        vertex_idx(6) += inf_step;
    }

    void cubeInflation_cpu(VectorXi& vertex_idx_lst, VectorXi& vertex_idx) {   
        int iter = 0;
        while(iter < itr_inflate_max) {   
            vertex_idx_lst = vertex_idx;

            inflateY_n(vertex_idx);
            inflateY_p(vertex_idx);
            inflateX_n(vertex_idx);
            inflateX_p(vertex_idx);

            bool is_inflate_conti = false;
            for(int vtx = 0; vtx < 4; vtx++) {
                if((vertex_idx_lst(vtx) != vertex_idx(vtx)) || 
                   (vertex_idx_lst(vtx + 4) != vertex_idx(vtx + 4))) {
                    is_inflate_conti = true;
                    break;
                }
            }

            if(is_inflate_conti == false) break;

            for(int vtx = 0; vtx < 4; vtx++) { 
                vertex_idx_lst(vtx + 0) = vertex_idx(vtx + 0);
                vertex_idx_lst(vtx + 4) = vertex_idx(vtx + 4);
            }
            iter++;
        }
    }

    bool serialConvexTest2D(int can_x_index, int can_y_index,
                            int cluster_grid_num, int max_y_id,
                            const MatrixXi &cluster_xy_id,
                            const MatrixXi &inside_data,
                            const MatrixXi &map_data) {
        Vector2i index_1_med(can_x_index / 2, can_y_index / 2);
        Vector2i index_med, index_2;
        
        for (int i = cluster_grid_num - 1; i >= 0; i--) {
            index_2(0) = cluster_xy_id(i, 0);
            index_2(1) = cluster_xy_id(i, 1);

            if (inside_data(can_x_index, can_y_index) == 0) {
                index_med = index_1_med + (index_2/2);

                if (inside_data(index_med(0), index_med(1)) == 1)
                    continue;

                int x = can_x_index, y = can_y_index;
                int endX = index_2(0), endY = index_2(1);
                int dx = endX - x, dy = endY - y;
                int stepX = signum_cpu(dx), stepY = signum_cpu(dy);

                float tMaxX = intbound_cpu(0.5, dx);
                float tMaxY = intbound_cpu(0.5, dy);
                float tDeltaX = ((float)stepX) / dx;
                float tDeltaY = ((float)stepY) / dy;
                
                while (true) {   
                    if (tMaxX < tMaxY) {
                        x += stepX;
                        tMaxX += tDeltaX;
                    }
                    else if(tMaxY < tMaxX) {
                        y += stepY;
                        tMaxY += tDeltaY;
                    }
                    else {
                        y += stepY;
                        x += stepX;
                        tMaxY += tDeltaY;
                        tMaxX += tDeltaX;
                    }

                    if (x == endX && y == endY) break;
                    
                    if (inside_data(x, y) == 1) continue;
                    else if (map_data(x, y) == 1) return false;
                }
            }
        }
        return true;
    }
    
    void polytopeCluster_cpu(int& cluster_grid_num, int& active_grid_num) {   
        int candidate_grid_num;
        int itr_cluster_cnt = 0;
        
        while(itr_cluster_cnt < itr_cluster_max) {   
            candidate_grid_num = 0;
            for(int i = 0; i < active_grid_num; i++) {
                Eigen::Vector2i cur_cell(active_xy_id(i, 0), active_xy_id(i, 1));
                use_data(cur_cell.x(), cur_cell.y()) = 1;

                for(int dx = -1; dx < 2; dx++) { 
                    for(int dy = -1; dy < 2; dy++) {   
                        if(dx == 0 && dy == 0) continue;
                        
                        Eigen::Vector2i nei_cell = cur_cell + Eigen::Vector2i(dx, dy);

                        if(nei_cell.x() < 0 || nei_cell.x() > _max_x_id - 1 
                        || nei_cell.y() < 0 || nei_cell.y() > _max_y_id - 1)
                            continue;

                        if(map_data(nei_cell.x(), nei_cell.y()) == 1 
                        || use_data(nei_cell.x(), nei_cell.y()) == 1 
                        || invalid_data(nei_cell.x(), nei_cell.y()) == 1 
                        || inside_data(nei_cell.x(), nei_cell.y()) == 1) {
                            continue;
                        }
                        else {   
                            candidate_cell(candidate_grid_num, 0) = nei_cell.x();
                            candidate_cell(candidate_grid_num, 1) = nei_cell.y();
                            candidate_grid_num++;
                            use_data(nei_cell.x(), nei_cell.y()) = 1;
                        }
                    }
                }
            }

            if(candidate_grid_num == 0) break;

            active_grid_num = 0;
            for(int i = 0; i < candidate_grid_num; i++) {
                int cand_x = candidate_cell(i, 0);
                int cand_y = candidate_cell(i, 1);

                if(serialConvexTest2D(cand_x, cand_y, 
                                cluster_grid_num, _max_y_id, 
                                cluster_xy_id, inside_data, map_data)) {   
                    cluster_xy_id(cluster_grid_num, 0) = cand_x;
                    cluster_xy_id(cluster_grid_num, 1) = cand_y;

                    active_xy_id(active_grid_num, 0) = cand_x;
                    active_xy_id(active_grid_num, 1) = cand_y;

                    cluster_grid_num++;
                    active_grid_num++;

                    inside_data(cand_x, cand_y) = 0;
                }
                else {   
                    invalid_data(cand_x, cand_y) = 1;
                }
            }
            
            if(active_grid_num == 0) break;
            itr_cluster_cnt++; 
        }
    }

    void polygonGeneration(vector<int>& cluster_x_idx, vector<int>& cluster_y_idx) {   
        flagClear();
        
        if(cluster_x_idx.size() == 1) {
            Eigen::Vector2i single_point(cluster_x_idx[0], cluster_y_idx[0]);
            for(int vtx = 0; vtx < 4; vtx++) {   
                vertex_idx(vtx) = single_point.x();
                vertex_idx(vtx + 4) = single_point.y();
            }       
        }
        else {
            Eigen::Vector2i min_corner(100000, 100000);
            Eigen::Vector2i max_corner(-100000, -100000);
            
            for(int i = 0; i < (int)cluster_x_idx.size(); i++) {
                Eigen::Vector2i point(cluster_x_idx[i], cluster_y_idx[i]);
                min_corner = min_corner.cwiseMin(point);
                max_corner = max_corner.cwiseMax(point);
            }

            setVertexInitIndex(vertex_idx, min_corner.x(), min_corner.y(), 
                            max_corner.x(), max_corner.y());
        }

        for(int i = 0; i < 8; i++) {
            vertex_idx_lst(i) = vertex_idx(i);
        }

        cluster_x_idx.clear(); 
        cluster_y_idx.clear();

        cubeInflation_cpu(vertex_idx_lst, vertex_idx);
        

        std::vector<int> rect_grid_x, rect_grid_y;

        getGridsInRect(vertex_idx, rect_grid_x, rect_grid_y);           // based on the operation performed by cubeInflation_cpu , vertex_idx get updated , 
                                                                        // based on that rect_grid_x , rect_grid_y is updated

        
        std::vector<Eigen::Vector2i> rect_outside_grids;

        if(rect_grid_x.size() == 1) {
            rect_outside_grids.push_back(Eigen::Vector2i(rect_grid_x[0], rect_grid_y[0]));
        }

        else {
            for(int i = 0; i < (int)rect_grid_x.size(); i++) {   
                Eigen::Vector2i grid_cell(rect_grid_x[i], rect_grid_y[i]);
                use_data(grid_cell.x(), grid_cell.y()) = 1;
                bool is_boundary=false;
                
                for(int dx = -1; dx < 2; dx++) {   
                    for(int dy = -1; dy < 2; dy++) {   
                        if(dx == 0 && dy == 0) continue;

                        Eigen::Vector2i neighbor = grid_cell + Eigen::Vector2i(dx, dy);

                        if(neighbor.x() >= 0 && neighbor.x() < _max_x_id && 
                        neighbor.y() >= 0 && neighbor.y() < _max_y_id) {
                            if(inside_data(neighbor.x(), neighbor.y())!=2) {
                                is_boundary=true;
                                break;
                            }
                        }
                        else {
                            is_boundary= true;
                            break;
                        }
                    }
                    if(is_boundary) break;
                }

                if(is_boundary) {
                    rect_outside_grids.push_back(grid_cell);
                }
            }
        }
        
        for(const auto& grid : rect_outside_grids) {
            inside_data(grid.x(), grid.y()) = 0;
        }

        int init_cluster_grid_num = rect_outside_grids.size();
        int active_grid_num = init_cluster_grid_num;

        for(int i = 0; i < init_cluster_grid_num; i++) {
            cluster_xy_id(i, 0) = rect_outside_grids[i].x();
            cluster_xy_id(i, 1) = rect_outside_grids[i].y();

            active_xy_id(i, 0) = rect_outside_grids[i].x();
            active_xy_id(i, 1) = rect_outside_grids[i].y();
        }

        Eigen::Vector2i vertex_diff_1(
            vertex_idx(3) - vertex_idx(1),
            vertex_idx(7) - vertex_idx(5)
        );

        if(vertex_diff_1.x() == 0 || vertex_diff_1.y() == 0) { 

            for(int i = 0; i < init_cluster_grid_num; i++) {
                cluster_x_idx.push_back(cluster_xy_id(i, 0));
                cluster_y_idx.push_back(cluster_xy_id(i, 1));
            }

            for (int i = 0; i < rect_grid_x.size(); ++i) {
                cluster_x_idx.push_back(rect_grid_x[i]);
                cluster_y_idx.push_back(rect_grid_y[i]);
            }


            return;
        }

        int cluster_grid_num = init_cluster_grid_num;
        polytopeCluster_cpu(cluster_grid_num, active_grid_num);

        // Add INSIDE rectangle points FIRST
        for (int i = 0; i < rect_grid_x.size(); ++i) {
            cluster_x_idx.push_back(rect_grid_x[i]);
            cluster_y_idx.push_back(rect_grid_y[i]);
        }


        for(int i = 0; i < cluster_grid_num; i++) {
            cluster_x_idx.push_back(cluster_xy_id(i, 0));
            cluster_y_idx.push_back(cluster_xy_id(i, 1));
        }                                                        // now finally inside_data ( coordinates inside the rectangle is finally included)
                                                                 // inside the cluster data 
    }

    void loadMapData(const vector<uint8_t>& data) {
        if(data.size() != (size_t)_grid_num) {
            cerr << "Error: Map data size mismatch!" << endl;
            return;
        }
        for(int i = 0; i < _max_x_id; i++) {
            for(int j = 0; j < _max_y_id; j++) {
                map_data(i, j) = data[i * _max_y_id + j];
            }
        }
    }

    void printMapData() {
        cout << "\nMap Data (" << _max_x_id << "x" << _max_y_id << " grid - 1=obstacle, 0=free):" << endl;
        for(int i = 0; i < _max_x_id; i++) {
            for(int j = 0; j < _max_y_id; j++) {
                cout << map_data(i, j) << " ";
            }
            cout << endl;
        }
    }

    void visualizePolygon(const vector<int>& cluster_x, const vector<int>& cluster_y, int polygon_id) {
        MatrixXi viz_grid = map_data;
        
        for(size_t i = 0; i < cluster_x.size(); i++) {
            viz_grid(cluster_x[i], cluster_y[i]) = 3;
        }
        
        cout << "\n=== POLYGON " << polygon_id << " ===" << endl;
        cout << "Total vertices: " << cluster_x.size() << endl;
        cout << "Visualization (0=free, 1=obstacle, 3=polygon_vertex):" << endl;
        
        for(int i = 0; i < _max_x_id; i++) {
            for(int j = 0; j < _max_y_id; j++) {
                if(viz_grid(i, j) == 3) {
                    cout << "P"<<" ";
                } 
                else if(inside_data(i, j) == 2) {
                     cout << "P"<<" ";

                }else {
                    cout << viz_grid(i, j) << " ";
                }
            }
            cout << endl;
        }
        
        cout << "\nPolygon vertices:" << endl;
        for(size_t i = 0; i < cluster_x.size(); i++) {
            cout << "  (" << cluster_x[i] << ", " << cluster_y[i] << ")";
            if((i + 1) % 6 == 0) cout << endl;
        }
        cout << endl;
    }

    int getMaxX() { return _max_x_id; }
    int getMaxY() { return _max_y_id; }

private:
    int _max_x_id;
    int _max_y_id;
    int _grid_num;
    double _resolution;
    int itr_inflate_max;
    int itr_cluster_max;
    int inf_step;
    int _cluster_buffer_size;
    int _candidate_buffer_size;
    int _cluster_buffer_size_square;
    
    MatrixXi map_data;
    MatrixXi use_data;
    MatrixXi invalid_data;
    MatrixXi inside_data;
    VectorXi vertex_idx;
    VectorXi vertex_idx_lst;
    
    MatrixXi active_xy_id;
    MatrixXi cluster_xy_id;
    MatrixXi candidate_cell;

    inline float signum_cpu(const int &x) {
        return x == 0 ? 0 : x < 0 ? -1.0f : 1.0f;
    }

    inline float mod_cpu(const float &value, const float &modulus) {
        return fmod(fmod(value, modulus) + modulus, modulus);
    }

    inline float intbound_cpu(float s, int ds) {
        if (ds == 0) return numeric_limits<float>::max();
        else if (ds < 0) return intbound_cpu(-s, -ds);
        else {
            s = mod_cpu(s, 1.0f);
            return (1 - s) / ds;
        }
    }
};

// Helper function to extract seed voxels from continuous waypoints
vector<pair<int, int>> extractSeedVoxelsFromPath(
    const vector<pair<double, double>>& waypoints,
    int max_x, int max_y, double spacing = 1.0) {
    
    vector<pair<int, int>> seed_voxels;
    
    for(size_t i = 0; i < waypoints.size(); i++) {
        int x = static_cast<int>(round(waypoints[i].first));
        int y = static_cast<int>(round(waypoints[i].second));
        
        // Clamp to grid bounds
        x = max(0, min(x, max_x - 1));
        y = max(0, min(y, max_y - 1));
        
        // Add unique seed points
        if(seed_voxels.empty() || 
           seed_voxels.back().first != x || 
           seed_voxels.back().second != y) {
            seed_voxels.push_back({x, y});
        }
    }
    
    return seed_voxels;
    }
    class LidarPolygonGenerator : public rclcpp::Node {
private:
    polyhedronGenerator poly_gen;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
    
    // NEW: H-matrix publisher
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr h_matrix_pub_;
    
    std::mutex map_mutex;
    std::vector<uint8_t> latest_map_data;
    bool new_map_received = false;
    std::vector<Eigen::MatrixXd> corridor_H_matrices_;
    std::vector<Eigen::VectorXd> corridor_h_vectors_;
    
public:
    LidarPolygonGenerator() : Node("lidar_polygon_generator"), 
                              poly_gen(100, 100) {  // 100x100 grid
        
        // Configure parameters
        poly_gen.paramSet(false, false, true, 100, 100, 1.0, 100, 50);
        
        // Subscribe to occupancy grid
        occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/occupancy_grid_map", 10, std::bind(&LidarPolygonGenerator::occupancyCallback, this, std::placeholders::_1));
        
        // NEW: Publish consolidated H-matrix
        h_matrix_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/corridor_h_matrix", 10);
        
        RCLCPP_INFO(this->get_logger(), "LidarPolygonGenerator started.");
        RCLCPP_INFO(this->get_logger(), "Publishing H-matrix to /corridor_h_matrix");
    }

    // NEW: Publish consolidated H-matrix as ROS2 message
    void publishHMatrix(const Eigen::MatrixXd& H, const Eigen::VectorXd& h) {
        auto msg = std_msgs::msg::Float32MultiArray();
        
        // Message format: [num_constraints, H_rows, H_cols, h_size, H_data..., h_data...]
        msg.data.push_back(static_cast<float>(H.rows()));  // num_constraints
        msg.data.push_back(static_cast<float>(H.rows()));  // H_rows  
        msg.data.push_back(static_cast<float>(H.cols()));  // H_cols
        msg.data.push_back(static_cast<float>(h.size()));  // h_size
        
        // Set dimensions for easy parsing
        msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        msg.layout.dim[0].label = "data";
        msg.layout.dim[0].size = 4 + H.size() + h.size();
        msg.layout.dim[0].stride = 1;
        msg.layout.data_offset = 0;
        
        // Append H matrix (row-major)
        for(int i = 0; i < H.rows(); i++) {
            for(int j = 0; j < H.cols(); j++) {
                msg.data.push_back(static_cast<float>(H(i,j)));
            }
        }
        
        // Append h vector
        for(int i = 0; i < h.size(); i++) {
            msg.data.push_back(static_cast<float>(h(i)));
        }
        
        h_matrix_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published H-matrix: %zu constraints", H.rows());
    }

    void addCorridorConstraint(const Eigen::MatrixXd& H, const Eigen::VectorXd& h) {
        corridor_H_matrices_.push_back(H);
        corridor_h_vectors_.push_back(h);
    }
    
    void clearCorridorConstraints() {
        corridor_H_matrices_.clear();
        corridor_h_vectors_.clear();
    }

    void printConsolidatedHMatrix() {
        if(corridor_H_matrices_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No H-representations available!");
            return;
        }
        
        // Consolidate all H-representations into single matrix
        size_t total_rows = 0;
        for(const auto& H : corridor_H_matrices_) {
            total_rows += H.rows();
        }
        
        Eigen::MatrixXd consolidated_H(total_rows, 2);
        Eigen::VectorXd consolidated_h(total_rows);
        
        size_t row_offset = 0;
        for(size_t i = 0; i < corridor_H_matrices_.size(); i++) {
            const auto& H = corridor_H_matrices_[i];
            const auto& h = corridor_h_vectors_[i];
            
            consolidated_H.block(row_offset, 0, H.rows(), 2) = H;
            consolidated_h.segment(row_offset, h.size()) = h;
            row_offset += H.rows();
        }
        
        // PRINT TO CONSOLE (keep existing behavior)
        RCLCPP_INFO(this->get_logger(), "\n=== CONSOLIDATED H-REPRESENTATION (Hx <= h) ===");
        RCLCPP_INFO(this->get_logger(), "Total constraints: %zu rows", total_rows);
        for(size_t i = 0; i < std::min<size_t>(10, total_rows); i++) {
            RCLCPP_INFO(this->get_logger(), "%4zu: [%8.3f, %8.3f, %8.3f]", 
                    i, consolidated_H(i, 0), consolidated_H(i, 1), consolidated_h(i));
        }
        if(total_rows > 10) {
            RCLCPP_INFO(this->get_logger(), "... (%zu total constraints)", total_rows);
        }
        
        // NEW: PUBLISH TO ROS2 TOPIC
        publishHMatrix(consolidated_H, consolidated_h);
    }

    void occupancyCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex);
        
        if(msg->info.width != 100 || msg->info.height != 100) {
            RCLCPP_WARN(this->get_logger(), 
                       "Expected 100x100 grid, got %dx%d. Skipping.",
                       msg->info.width, msg->info.height);
            return;
        }
        
        latest_map_data.resize(msg->data.size());
        for (size_t i = 0; i < msg->data.size(); ++i) {
            int8_t occ = msg->data[i];
            if (occ == -1) {
                latest_map_data[i] = 2;
            } else if (occ >= 50) {
                latest_map_data[i] = 1;
            } else {
                latest_map_data[i] = 0;
            }
        }
        new_map_received = true;
    }
    
    bool getLatestMap(std::vector<uint8_t>& map_data) {
        std::lock_guard<std::mutex> lock(map_mutex);
        if(new_map_received) {
            map_data = latest_map_data;
            new_map_received = false;
            return true;
        }
        return false;
    }
    
    polyhedronGenerator& getPolyGen() { return poly_gen; }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    cout << "========================================" << endl;
    cout << "  CONTINUOUS CORRIDOR GENERATOR" << endl;
    cout << "  Real-time H-representation updates" << endl;
    cout << "========================================\n" << endl;
    
    auto lidar_gen = std::make_shared<LidarPolygonGenerator>();
    const int GRID_SIZE = 100;
    QuickHull<double> qh;
    
    int iteration = 0;
    std::vector<uint8_t> occupancy_map;
    
    RCLCPP_INFO(lidar_gen->get_logger(), "🚀 Continuous corridor generator started!");
    RCLCPP_INFO(lidar_gen->get_logger(), "Waiting for first /occupancy_grid_map...");
    
    while(rclcpp::ok()) {
        rclcpp::spin_some(lidar_gen);
        
        if(lidar_gen->getLatestMap(occupancy_map)) {
            iteration++;
            cout << "\n" << string(60, '=') << endl;
            cout << "=== ITERATION " << iteration << " - NEW MAP RECEIVED ===" << endl;
            cout << string(60, '=') << endl;
            
            // Load map into poly_gen
            polyhedronGenerator& poly_gen = lidar_gen->getPolyGen();
            poly_gen.loadMapData(occupancy_map);
            
            cout << "Map loaded (" << GRID_SIZE << "x" << GRID_SIZE << ")" << endl;
            
            // H* path planning
            HStarPlanner planner;
            planner.init(GRID_SIZE, GRID_SIZE, 1.0, 16, 2.0);
            
            for(int gx = 0; gx < GRID_SIZE; gx++) {
                for(int gy = 0; gy < GRID_SIZE; gy++) {
                    if(occupancy_map[gx * GRID_SIZE + gy] == 1) {
                        planner.setObstacle(gx, gy);
                    }
                }
            }
            
            int start_x = 3, start_y = 3;
            int goal_x = 45, goal_y = 45;
            
            vector<Vec2> path = planner.search(start_x, start_y, 0.0, goal_x, goal_y, 0.0);
            if(path.empty()) {
                RCLCPP_WARN(lidar_gen->get_logger(), "H* path not found, skipping...");
                continue;
            }
            
            // Extract seeds & generate corridors
            vector<pair<double,double>> path_waypoints;
            for(auto &p : path) path_waypoints.push_back({p.x, p.y});
            
            vector<pair<int, int>> seed_voxels = extractSeedVoxelsFromPath(
                path_waypoints, GRID_SIZE, GRID_SIZE);
            
            lidar_gen->clearCorridorConstraints();
            
            vector<vector<int>> all_polygon_x, all_polygon_y;
            for(size_t i = 0; i < seed_voxels.size(); i++) {
                vector<int> seed_x = {seed_voxels[i].first};
                vector<int> seed_y = {seed_voxels[i].second};
                
                poly_gen.polygonGeneration(seed_x, seed_y);
                all_polygon_x.push_back(seed_x);
                all_polygon_y.push_back(seed_y);
                
                // Compute H-representation
                const auto& poly_x = all_polygon_x.back();
                const auto& poly_y = all_polygon_y.back();
                
                if(poly_x.size() == poly_y.size() && poly_x.size() > 2) {
                    std::vector<quickhull::Vector2<double>> points2;
                    for(size_t j = 0; j < poly_x.size(); ++j) {
                        points2.emplace_back(static_cast<double>(poly_x[j]), static_cast<double>(poly_y[j]));
                    }
                    
                    auto hull = qh.getConvexHull(points2.data(), points2.size(), true, false);
                    std::vector<LineEquations::Point> hull_points;
                    
                    for(const auto &v : hull.vertices()) {
                        int xi = static_cast<int>(std::round(v.x));
                        int yi = static_cast<int>(std::round(v.y));
                        hull_points.emplace_back(xi, yi);
                    }
                    
                    LineEquations le(hull_points);
                    le.compute();
                    
                    Eigen::MatrixXd A = le.A();
                    Eigen::VectorXd b = le.b();
                    
                    Eigen::Vector2d seed_pt(seed_voxels[i].first, seed_voxels[i].second);
                    Eigen::MatrixXd A_in = A;
                    Eigen::VectorXd b_in = b;
                    
                    for(int j = 0; j < A.rows(); ++j) {
                        if(A.row(j).dot(seed_pt) > b(j)) {
                            A_in.row(j) *= -1.0;
                            b_in(j) *= -1.0;
                        }
                    }
                    
                    lidar_gen->addCorridorConstraint(A_in, b_in);
                }
            }
            
            // Print consolidated H-matrix
            lidar_gen->printConsolidatedHMatrix();
            
            RCLCPP_INFO(lidar_gen->get_logger(), "✓ Iteration %d complete! Ready for next map...", iteration);
            cout << string(60, '=') << "\n" << endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz polling
    }
    
    rclcpp::shutdown();
    return 0;
}
