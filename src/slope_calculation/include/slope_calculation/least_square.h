#include <Eigen/Dense>
#include <iostream>

class LeastSquare
{
public:
    LeastSquare() {}
    ~LeastSquare() {}

public:
    void init(int points_num, int degree);
    void doSolve(const Eigen::VectorXd &x, const Eigen::VectorXd &y);
    Eigen::VectorXd getCoeffs() { return coeffs_; }

private:
    double relatePow(const Eigen::VectorXd &x, int points_num, int degree);
    double relateMultiXY(const Eigen::VectorXd &x, const Eigen::VectorXd &y, int points_num, int degree);

    int degree_;
    int points_num_;
    Eigen::MatrixXd A_;
    Eigen::VectorXd b_;
    Eigen::VectorXd coeffs_;
};
