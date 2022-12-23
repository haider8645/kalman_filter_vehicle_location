#include <iostream>
#include <vector>
#include <matplot/matplot.h>
#include "FilterPosition2D.hpp"

using namespace matplot;

int main(int argc, char** argv)
{
    kalman_filter_robot_location::FilterPosition2D filter2D;

    //For theoretical background and explanation, see: https://www.kalmanfilter.net/multiExamples.html 
    //The values for H,F,Q, and P are assigned based on the vehicle location example from the webpage sourced above. 
    Eigen::MatrixXd F(6,6),Q(6,6),P(6,6);
    Eigen::MatrixXd R(2,2);
    Eigen::MatrixXd H(R.rows(),F.rows());
    H << 1,0,0,0,0,0,
         0,0,0,1,0,0;
    F << 1,1,0.5,0,0,0,
         0,1,1,0,0,0,
         0,0,1,0,0,0,
         0,0,0,1,1,0.5,
         0,0,0,0,1,1,
         0,0,0,0,0,1;
    Q << 0.25,0.5,0.5,0,0,0,
         0.5,1,1,0,0,0,
         0.5,1,1,0,0,0,
         0,0,0,0.25,0.5,0.5,
         0,0,0,0.5,1,1,
         0,0,0,0.5,1,1;
    P << 500,0,0,0,0,0,
         0,500,0,0,0,0,
         0,0,500,0,0,0,
         0,0,0,500,0,0,
         0,0,0,0,500,0,
         0,0,0,0,0,500;     
    Eigen::VectorXd x(6,1);
    x << 0,0,0,0,0,0;

    //Create some dummy measurements which follow the sinusoidal pattern
    const uint measurement_samples = 100;
    std::vector<Eigen::Vector2d> measurements;
    for (int i{0}; i < measurement_samples; ++i){
         measurements.push_back({i,sin(i)});
    }
    //The example on the webpage linked above assumes a straight line motion with a turn. Our measurements follow the sinusoidal pattern
    //and include sharp turns which make the initial assumption of a constant accelerating system false. To compensate for this, we can
    //assign a large variance to the process noise matrix and a lower value to the measurement matrix. 
    Q = 0.04*Q; // Acceleration variance = 2 m/s^2      
    R << 1,0,   // Measurement variance = 1 m    
         0,1; 
    //You can try to increase the R value to the one shown in the example and see that the filter diverges after a few turns. The reason is that
    //our model is a linear model so its predicaton is not accurate which means that it has a high process noise. Now, on top of that, if we have
    //a high measurement variable as well like R = [9 0; 0 9] then the filter diverges.     

    filter2D.initFilterParams(F,Q,R,H);
    filter2D.initFilterState(x,P);

    std::vector<double> measurement_x; 
    std::vector<double> measurement_y; 
    std::vector<double> result_x; 
    std::vector<double> result_y; 

    for (int i{0}; i < measurements.size(); ++i){
        Eigen::VectorXd sample(2,1);
        sample << measurements.at(i).x(), measurements.at(i).y();  
        filter2D.update(sample);
        Eigen::VectorXd state = filter2D.getState();
        
        measurement_x.push_back(sample(0)); 
        measurement_y.push_back(sample(1)); 
        result_x.push_back(state(0));      
        result_y.push_back(state(3));    
    }

    plot(measurement_x, measurement_y, "-o");
    hold(on);
    plot(result_x, result_y, "-");
    show();

    return 0;
}
