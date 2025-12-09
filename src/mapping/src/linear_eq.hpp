#ifndef LINE_EQUATIONS_HPP
#define LINE_EQUATIONS_HPP

#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <Eigen/Dense>

namespace line_eq
{

class LineEquations
{
public:
    using Point = std::pair<int, int>;

    LineEquations() = default;

    explicit LineEquations(const std::vector<Point>& points)
    {
        setPoints(points);
    }

    // Set / update polygon points
    void setPoints(const std::vector<Point>& points)
    {
        points_ = points;
        computed_ = false;
    }

    // ----------------------------------------------------
    // Compute A x = b from polygon edges
    // Each edge: a x + b y + c = 0  ->  [a b] x = -c
    // ----------------------------------------------------
    void compute()
    {
        equations_.clear();

        const int n = static_cast<int>(points_.size());
        if (n < 2)
        {
            A_.resize(0, 2);
            b_.resize(0);
            computed_ = true;
            return;
        }

        A_.resize(n, 2);
        b_.resize(n);

        for (int i = 0; i < n; ++i)
        {
            double x1 = points_[i].first;
            double y1 = points_[i].second;

            double x2 = points_[(i + 1) % n].first;
            double y2 = points_[(i + 1) % n].second;

            // Line coefficients
            double a = y1 - y2;
            double b = x2 - x1;
            double c = x1 * y2 - x2 * y1;

            // Fill Ax = b form
            A_(i, 0) = a;
            A_(i, 1) = b;
            b_(i)    = -c;

            // Human-readable equation
            equations_.emplace_back(
                std::to_string((int)a) + "x + " +
                std::to_string((int)b) + "y = " +
                std::to_string((int)(-c))
            );
        }

        computed_ = true;
    }

    // ✅ Accessors
    const Eigen::MatrixXd& A() const { return A_; }
    const Eigen::VectorXd& b() const { return b_; }
    const std::vector<std::string>& equations() const { return equations_; }

    // ✅ Print in Ax = b form
    void print(std::ostream& os = std::cout) const
    {
        if (!computed_)
        {
            os << "Warning: LineEquations::compute() not called yet.\n";
        }

        os << "Matrix A (coefficients):\n";
        os << A_ << "\n\n";

        os << "Vector b (constants):\n";
        os << b_ << "\n\n";

        os << "Equations (Ax = b):\n";
        for (const auto& eq : equations_)
            os << eq << "\n";
    }

private:
    std::vector<Point> points_;

    Eigen::MatrixXd A_;      // ✅ N×2 matrix [a b]
    Eigen::VectorXd b_;      // ✅ N×1 vector [-c]
    std::vector<std::string> equations_;

    bool computed_ = false;
};

} // namespace line_eq

#endif // LINE_EQUATIONS_HPP
