#ifndef _FILTER_POSITION_2D_HPP_
#define _FILTER_POSITION_2D_HPP_

#include <eigen3/Eigen/Dense>

namespace kalman_filter_robot_location
{
class FilterPosition2D
{
public:
    FilterPosition2D();
    void initFilterParams(Eigen::MatrixXd& F, Eigen::MatrixXd& Q, Eigen::MatrixXd& R, Eigen::MatrixXd& H);
    void initFilterState(Eigen::VectorXd x_0, Eigen::MatrixXd& P_0);
    void update(Eigen::VectorXd& measurement);
    Eigen::VectorXd getState();

private:

    void predict();

    Eigen::MatrixXd F_, P_, Q_, I_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    Eigen::VectorXd x_;
};

} // end namespace kalman_filter_robot_location

#endif // _FILTER_POSITION_2D_HPP_
