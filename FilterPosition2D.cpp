#include "FilterPosition2D.hpp"
#include <iostream>

using namespace std;
using namespace kalman_filter_robot_location;

FilterPosition2D::FilterPosition2D(){

}

void FilterPosition2D::initFilterParams(Eigen::MatrixXd& F, Eigen::MatrixXd& Q, Eigen::MatrixXd& R, Eigen::MatrixXd& H){
    F_ = F;
    Q_ = Q;
    R_ = R;
    H_ = H;
    I_ = Eigen::MatrixXd(F_.rows(),F_.rows());
    I_.setIdentity();
}
void FilterPosition2D::initFilterState(Eigen::VectorXd x_0, Eigen::MatrixXd& P_0){
    x_ = x_0;
    P_ = P_0;    
}

void FilterPosition2D::predict(){
    x_ = F_ * x_;
    P_ =  F_ * P_ * F_.transpose() + Q_;
}

void FilterPosition2D::update(Eigen::VectorXd& measurement){
    predict();
    Eigen::MatrixXd K_gain = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();
    std::cout << K_gain << std::endl;
    x_ = x_ + K_gain*(measurement - H_*x_);
    P_ = (I_ - K_gain*H_)*P_*(I_*K_gain*H_).transpose() + K_gain * R_ * K_gain.transpose();
}

Eigen::VectorXd FilterPosition2D::getState(){
    return x_;
}