#include "least_square.h"

void LeastSquare::init(int points_num, int degree)
{
    points_num_ = points_num;
    degree_ = degree;
    A_.resize(degree + 1, degree + 1);
    b_.resize(degree + 1);
}

void LeastSquare::doSolve(const Eigen::VectorXd &x, const Eigen::VectorXd &y)
{
    for (int i = 0; i < degree_ + 1; i++) {
        for (int j = 0; j < degree_ + 1; j++) {
            A_(i, j) = relatePow(x, points_num_, i + j);
        }
        b_(i) = relateMultiXY(x, y, points_num_, i);
    }

    A_(0, 0) = points_num_;

    coeffs_ = A_.colPivHouseholderQr().solve(b_);
}

double LeastSquare::relatePow(const Eigen::VectorXd &x, int points_num, int degree)
{
    double sum_pow = 0;
    for (int i = 0; i < points_num; i++) {
        sum_pow += pow(x(i), degree);
    }

    return sum_pow;
}

double LeastSquare::relateMultiXY(const Eigen::VectorXd &x, const Eigen::VectorXd &y, int points_num, int degree)
{
    double sum_multi_xy = 0;
    for (int i = 0; i < points_num; i++) {
        sum_multi_xy += pow(x(i), degree) * y(i);
    }
    return sum_multi_xy;
}