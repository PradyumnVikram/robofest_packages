// hstar_2d_final.hpp
#ifndef HSTAR_2D_FINAL_HPP
#define HSTAR_2D_FINAL_HPP



#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <cstring>
#include <iostream>
#include <algorithm>

using namespace std;

// LIGHTWEIGHT VECTOR STRUCTURES

struct Vec2 {
    double x, y;
    Vec2() : x(0), y(0) {}
    Vec2(double _x, double _y) : x(_x), y(_y) {}
    
    Vec2 operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 operator*(double s) const { return Vec2(x * s, y * s); }
    
    double norm() const { return sqrt(x*x + y*y); }
    double normSq() const { return x*x + y*y; }
    
    Vec2 normalized() const {
        double n = norm();
        return (n > 1e-9) ? Vec2(x/n, y/n) : Vec2(0, 0);
    }
    
    double dot(const Vec2& v) const { return x*v.x + y*v.y; }
};

struct Vec2i {
    int x, y;
    Vec2i() : x(0), y(0) {}
    Vec2i(int _x, int _y) : x(_x), y(_y) {}
    
    bool operator==(const Vec2i& v) const { return x == v.x && y == v.y; }
    bool operator!=(const Vec2i& v) const { return x != v.x || y != v.y; }
};

// H* NODE - COMPACT REPRESENTATION


struct HNode {
    int gx, gy;           // Grid coordinates
    int hbin;             // Heading bin index
    double cx, cy;        // World center coordinates
    double heading;       // Heading angle (radians)
    
    double g;             // Cost from start
    double f;             // Total cost (g + h)
    
    int parent;           // Parent node index (-1 for start)
    int8_t state;         // 0=unvisited, 1=open, -1=closed
    
    HNode() : gx(0), gy(0), hbin(0), cx(0), cy(0), heading(0),
              g(numeric_limits<double>::infinity()),
              f(numeric_limits<double>::infinity()),
              parent(-1), state(0) {}
};

// MOTION PRIMITIVE - DUBINS-LIKE MOTIONS


struct MotionPrimitive {
    double body_dx, body_dy;  // Body frame displacement
    double dheading;          // Heading change (radians)
    double cost;              // Motion cost
    
    MotionPrimitive(double dx, double dy, double dh, double c)
        : body_dx(dx), body_dy(dy), dheading(dh), cost(c) {}
};

// H* PLANNER CLASS - COMPETITION OPTIMIZED

class HStarPlanner {
private:
    // Grid parameters
    int max_x, max_y;
    double resolution;
    double inv_resolution;
    
    // Heading discretization
    int heading_bins;
    double heading_res;
    
    // Flattened arrays for efficiency
    uint8_t* occupancy;     // [max_x * max_y]
    HNode* nodes;           // [max_x * max_y * heading_bins]
    size_t node_count;
    
    // Motion primitives
    vector<MotionPrimitive> motions;
    double min_turn_radius;
    
    // Search parameters
    double tie_breaker;
    
    // Seed extraction parameters
    double seed_min_distance;
    double seed_angle_threshold;

public:
    HStarPlanner()
        : max_x(0), max_y(0), resolution(1.0), inv_resolution(1.0),
          heading_bins(16), heading_res(0.0),
          occupancy(nullptr), nodes(nullptr), node_count(0),
          min_turn_radius(2.0), tie_breaker(1.001),
          seed_min_distance(1.5), seed_angle_threshold(0.26) {}
    
    ~HStarPlanner() {
        cleanup();
    }
    
    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    
    void init(int grid_x, int grid_y,
              double res = 1.0,
              int h_bins = 16,
              double turn_radius = 2.0) {
        
        cleanup();
        
        max_x = grid_x;
        max_y = grid_y;
        resolution = res;
        inv_resolution = 1.0 / res;
        
        heading_bins = max(4, h_bins);
        heading_res = 2.0 * M_PI / heading_bins;
        min_turn_radius = max(0.5, turn_radius);
        
        // Allocate occupancy grid
        size_t grid_size = static_cast<size_t>(max_x) * max_y;
        occupancy = new uint8_t[grid_size];
        memset(occupancy, 0, grid_size * sizeof(uint8_t));
        
        // Allocate flattened node array
        node_count = grid_size * heading_bins;
        nodes = new HNode[node_count];
        initNodes();
        
        // Build motion primitives
        buildMotionPrimitives();
        
        cout << "[H*] Grid: " << max_x << "x" << max_y 
             << " @ " << resolution << "m, "
             << heading_bins << " heading bins ("
             << (heading_res * 180.0 / M_PI) << "° each)" << endl;
        cout << "[H*] Motion primitives: " << motions.size() << endl;
    }
    
    // ========================================================================
    // OBSTACLE MANAGEMENT
    // ========================================================================
    
    void setObstacle(int gx, int gy) {
        if (!inBounds(gx, gy)) return;
        occupancy[cellIndex(gx, gy)] = 1;
    }
    
    void clearObstacle(int gx, int gy) {
        if (!inBounds(gx, gy)) return;
        occupancy[cellIndex(gx, gy)] = 0;
    }
    
    void clearAllObstacles() {
        if (occupancy) {
            memset(occupancy, 0, max_x * max_y * sizeof(uint8_t));
        }
    }
    // MAIN SEARCH ALGORITHM
   
    vector<Vec2> search(int sx, int sy, double sheading,
                        int gx, int gy, double gheading = 0.0,
                        int max_iter = 200000) {
        
        if (!occupancy || !nodes) return {};
        if (!inBounds(sx, sy) || !inBounds(gx, gy)) {
            cout << "[H*] Start/goal out of bounds" << endl;
            return {};
        }
        if (isOccupied(sx, sy) || isOccupied(gx, gy)) {
            cout << "[H*] Start/goal in obstacle" << endl;
            return {};
        }
        
        // Normalize headings
        sheading = normalizeAngle(sheading);
        gheading = normalizeAngle(gheading);
        
        int sbin = angleToBin(sheading);
        
        // Reset all nodes
        resetNodes();
        
        // Priority queue
        struct PQItem { size_t idx; double f; };
        auto cmp = [](const PQItem& a, const PQItem& b) { return a.f > b.f; };
        priority_queue<PQItem, vector<PQItem>, decltype(cmp)> open(cmp);
        
        // Initialize start node
        size_t start_idx = nodeIndex(sx, sy, sbin);
        HNode* start = &nodes[start_idx];
        start->g = 0.0;
        start->f = heuristic(sx, sy, gx, gy);
        start->state = 1;
        start->parent = -1;
        
        open.push({start_idx, start->f});
        
        int iterations = 0;
        int expansions = 0;
        
        while (!open.empty() && iterations < max_iter) {
            iterations++;
            
            PQItem top = open.top();
            open.pop();
            
            size_t cur_idx = top.idx;
            HNode* cur = &nodes[cur_idx];
            
            // Skip if already closed (outdated queue entry)
            if (cur->state == -1) continue;
            
            // Goal test (position match, heading flexible)
            if (cur->gx == gx && cur->gy == gy) {
                cout << "[H*] Path found: " << iterations << " iterations, "
                     << expansions << " expansions" << endl;
                return reconstructPath(cur_idx);
            }
            
            // Mark as closed
            cur->state = -1;
            expansions++;
            
            // Expand neighbors using motion primitives
            for (const MotionPrimitive& mp : motions) {
                // Transform motion from body to world frame
                double cos_h = cos(cur->heading);
                double sin_h = sin(cur->heading);
                
                double world_dx = mp.body_dx * cos_h - mp.body_dy * sin_h;
                double world_dy = mp.body_dx * sin_h + mp.body_dy * cos_h;
                
                double next_cx = cur->cx + world_dx;
                double next_cy = cur->cy + world_dy;
                double next_heading = normalizeAngle(cur->heading + mp.dheading);
                
                // Convert to grid coordinates
                int next_gx = static_cast<int>(floor(next_cx / resolution + 0.5));
                int next_gy = static_cast<int>(floor(next_cy / resolution + 0.5));
                
                if (!inBounds(next_gx, next_gy)) continue;
                if (isOccupied(next_gx, next_gy)) continue;
                
                int next_hbin = angleToBin(next_heading);
                size_t next_idx = nodeIndex(next_gx, next_gy, next_hbin);
                HNode* neighbor = &nodes[next_idx];
                
                if (neighbor->state == -1) continue;
                
                double tentative_g = cur->g + mp.cost;
                
                if (neighbor->state != 1 || tentative_g < neighbor->g - 1e-9) {
                    neighbor->parent = static_cast<int>(cur_idx);
                    neighbor->g = tentative_g;
                    neighbor->f = tentative_g + tie_breaker * heuristic(next_gx, next_gy, gx, gy);
                    
                    if (neighbor->state != 1) {
                        neighbor->state = 1;
                    }
                    open.push({next_idx, neighbor->f});
                }
            }
        }
        
        cout << "[H*] No path found after " << iterations << " iterations" << endl;
        return {};
    }
    
    // SEED EXTRACTION - BRESENHAM + FILTERING
  
    
    vector<Vec2i> extractSeeds(const vector<Vec2>& path,
                               double min_dist = 1.5,
                               double angle_thresh = 0.26) {
        
        if (path.empty()) return {};
        
        // Step 1: Convert path to grid cells and apply Bresenham
        vector<Vec2i> voxels;
        voxels.reserve(path.size() * 2);
        
        Vec2i prev = worldToGrid(path[0]);
        voxels.push_back(prev);
        
        for (size_t i = 1; i < path.size(); i++) {
            Vec2i cur = worldToGrid(path[i]);
            bresenhamLine(prev.x, prev.y, cur.x, cur.y, voxels);
            prev = cur;
        }
        
        // Step 2: Remove duplicates
        vector<Vec2i> unique;
        unique.reserve(voxels.size());
        for (const Vec2i& v : voxels) {
            if (unique.empty() || unique.back() != v) {
                unique.push_back(v);
            }
        }
        
        // Step 3: Filter by distance and angle
        vector<Vec2i> seeds;
        if (unique.empty()) return seeds;
        
        seeds.push_back(unique[0]);
        Vec2 last_seed(unique[0].x, unique[0].y);
        Vec2 last_dir(0, 0);
        
        for (size_t i = 1; i < unique.size(); i++) {
            Vec2 cur(unique[i].x, unique[i].y);
            Vec2 delta = cur - last_seed;
            double dist = delta.norm();
            
            if (dist < min_dist * 0.5) continue;
            
            Vec2 cur_dir = delta.normalized();
            double angle_change = 0.0;
            
            if (last_dir.norm() > 1e-6) {
                double dot = cur_dir.dot(last_dir);
                dot = max(-1.0, min(1.0, dot));
                angle_change = acos(dot);
            }
            
            // Add seed if significant change
            if (angle_change > angle_thresh || dist >= min_dist) {
                seeds.push_back(unique[i]);
                last_seed = cur;
                last_dir = cur_dir;
            }
        }
        
        // Always include goal
        if (seeds.empty() || seeds.back() != unique.back()) {
            seeds.push_back(unique.back());
        }
        
        cout << "[H*] Extracted " << seeds.size() << " seeds from "
             << path.size() << " waypoints (" << unique.size() 
             << " unique voxels)" << endl;
        
        return seeds;
    }
    
    // GETTERS & SETTERS
  
    
    void setSeedMinDistance(double d) { seed_min_distance = d; }
    void setSeedAngleThreshold(double a) { seed_angle_threshold = a; }
    void setTieBreaker(double tb) { tie_breaker = tb; }
    
    int getMaxX() const { return max_x; }
    int getMaxY() const { return max_y; }
    double getResolution() const { return resolution; }
    
    bool isOccupied(int gx, int gy) const {
        if (!inBounds(gx, gy)) return true;
        return occupancy[cellIndex(gx, gy)] != 0;
    }
    
    Vec2 gridToWorld(const Vec2i& grid) const {
        return Vec2(grid.x * resolution, grid.y * resolution);
    }
    
    Vec2i worldToGrid(const Vec2& world) const {
        int gx = static_cast<int>(floor(world.x / resolution + 0.5));
        int gy = static_cast<int>(floor(world.y / resolution + 0.5));
        gx = max(0, min(max_x - 1, gx));
        gy = max(0, min(max_y - 1, gy));
        return Vec2i(gx, gy);
    }

private:
    // INITIALIZATION HELPERS
    
    void initNodes() {
        size_t idx = 0;
        for (int gx = 0; gx < max_x; gx++) {
            for (int gy = 0; gy < max_y; gy++) {
                for (int h = 0; h < heading_bins; h++) {
                    HNode& n = nodes[idx++];
                    n.gx = gx;
                    n.gy = gy;
                    n.hbin = h;
                    n.cx = gx * resolution;
                    n.cy = gy * resolution;
                    n.heading = h * heading_res;
                    n.g = numeric_limits<double>::infinity();
                    n.f = numeric_limits<double>::infinity();
                    n.parent = -1;
                    n.state = 0;
                }
            }
        }
    }
    
    void resetNodes() {
        for (size_t i = 0; i < node_count; i++) {
            nodes[i].g = numeric_limits<double>::infinity();
            nodes[i].f = numeric_limits<double>::infinity();
            nodes[i].parent = -1;
            nodes[i].state = 0;
        }
    }
    
    void buildMotionPrimitives() {
        motions.clear();
        
        double step = max(1.0, resolution);
        
        // Straight forward
        motions.emplace_back(step, 0.0, 0.0, step);
        
        // Arcs with different radii
        double rad = min_turn_radius;
        
        // Small turns (1 heading bin)
        double ang1 = heading_res;
        double arc_len1 = rad * ang1;
        double dx1 = rad * sin(ang1);
        double dy1 = rad * (1.0 - cos(ang1));
        
        motions.emplace_back(dx1, dy1, ang1, arc_len1);    // Left
        motions.emplace_back(dx1, -dy1, -ang1, arc_len1);  // Right
        
        // Medium turns (2 heading bins)
        double ang2 = 2.0 * heading_res;
        double arc_len2 = rad * ang2;
        double dx2 = rad * sin(ang2);
        double dy2 = rad * (1.0 - cos(ang2));
        
        motions.emplace_back(dx2, dy2, ang2, arc_len2);    // Left
        motions.emplace_back(dx2, -dy2, -ang2, arc_len2);  // Right
        
        // Sharp turns (3 heading bins)
        double ang3 = 3.0 * heading_res;
        double arc_len3 = rad * 0.8 * ang3;  // Slightly penalized
        double dx3 = rad * 0.8 * sin(ang3);
        double dy3 = rad * 0.8 * (1.0 - cos(ang3));
        
        motions.emplace_back(dx3, dy3, ang3, arc_len3);    // Left
        motions.emplace_back(dx3, -dy3, -ang3, arc_len3);  // Right
        
        // Normalize costs by resolution
        for (MotionPrimitive& m : motions) {
            m.cost = max(1e-6, m.cost / resolution);
        }
    }
    
    void cleanup() {
        if (occupancy) {
            delete[] occupancy;
            occupancy = nullptr;
        }
        if (nodes) {
            delete[] nodes;
            nodes = nullptr;
        }
        node_count = 0;
        max_x = max_y = 0;
    }
    
    // INDEX CALCULATION

    inline size_t cellIndex(int gx, int gy) const {
        return static_cast<size_t>(gx) * static_cast<size_t>( max_y) + static_cast<size_t> 
        (gy);
    }
    
    inline size_t nodeIndex(int gx, int gy, int hbin) const {
        return (static_cast<size_t>(gx) * static_cast<size_t>(max_y)
         + static_cast<size_t>(gy)) * static_cast<size_t>(heading_bins)
         + static_cast<size_t>(hbin);

    }
    
    inline bool inBounds(int gx, int gy) const {
        return gx >= 0 && gy >= 0 && gx < max_x && gy < max_y;
    }
    
    // ANGLE UTILITIES
    
    inline double normalizeAngle(double a) const {
        while (a < 0.0) a += 2.0 * M_PI;
        while (a >= 2.0 * M_PI) a -= 2.0 * M_PI;
        return a;
    }
    
    inline int angleToBin(double a) const {
        a = normalizeAngle(a);
        int bin = static_cast<int>(std::round(a / heading_res));
        if (bin >= heading_bins) bin = 0;
        if (bin < 0) bin = 0;
        return bin;
    }
    
    inline double heuristic(int ax, int ay, int bx, int by) const {
        double dx = ax - bx;
        double dy = ay - by;
        return sqrt(dx*dx + dy*dy);
    }
    
  
    // PATH RECONSTRUCTION
    
    vector<Vec2> reconstructPath(size_t goal_idx) const {
        vector<Vec2> path;
        int cur = static_cast<int>(goal_idx);
        
        while (cur >= 0) {
            const HNode& n = nodes[cur];
            path.push_back(Vec2(n.cx, n.cy));
            cur = n.parent;
        }
        
        reverse(path.begin(), path.end());
        cout << "[H*] Path length: " << path.size() << " waypoints" << endl;
        return path;
    }
    // BRESENHAM LINE ALGORITHM
    
    void bresenhamLine(int x0, int y0, int x1, int y1, vector<Vec2i>& out) const {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        
        int x = x0, y = y0;
        
        while (true) {
            out.push_back(Vec2i(x, y));
            if (x == x1 && y == y1) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
};

#endif // HSTAR_2D_FINAL_HPP