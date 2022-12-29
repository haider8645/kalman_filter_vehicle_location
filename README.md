# Kalman Filter to Track Vehicle Location
This repository is the implementation of the [Example 9 – vehicle location estimation](https://www.kalmanfilter.net/multiExamples.html). Thanks to them for making it easy to understand the kalman filter!  

# Pre-reqs
 1. Eigen library
>    sudo apt-get install libeigen3-dev
 2. Matplot++ (https://alandefreitas.github.io/matplotplusplus/) 
>    git clone https://github.com/alandefreitas/matplotplusplus/ \
>    cd matplotplusplus \
>    mkdir build && cd build \
>    cmake .. \
>    sudo make install 
3.   Kalman Filter Vehicle Location
>    git clone https://github.com/haider8645/kalman_filter_vehicle_location.git \
>    cd kalman_filter_vehicle_location \
>    mkdir build && cd build \
>    cmake .. \
>    make \
>    ./kalman_filter_robot_location_bin

### Note:
Tested on Ubuntu 20.04.

# Results
Try to play around with the process noise variance and the measurement noise variance to see the different behaviors of the filter.
Additionally you can try different measurements (linear and non-linear) to test the performance of filter.

#### Low Process Noise & Low Measurement Noise (Acceleration variance: 0.2 m/s2, Measurement variance: 1 m) 
Our assumption for the process model was that the system has zero acceleration (linear model) but our measurements are very curvy (non-linear) which go against the model assumption. A low process noise here means that the filter diverges becasue it was designed for a zero acceleration model.

![image](https://user-images.githubusercontent.com/23505408/209317521-d2c5005c-ab18-4ec2-8522-160316c33331.png)

#### Low Process Noise & High Measurement Noise (Acceleration variance: 0.2 m/s2, Measurement variance: 3 m) 
The low process noise was already making the filter diverge, a high measurement noise makes it diverge even faster...
![image](https://user-images.githubusercontent.com/23505408/209317896-6d0a31ea-ae15-421b-a4d4-d1f1f250bdb7.png)

#### High Process Noise & Low Measurement Noise (Acceleration variance: 2 m/s2, Measurement variance: 1 m) 

Better tracking because the process noise is high. The process noise is high because of the choice of sinusoidal measurements (non-linear) which does not fit well with the choice of a linear process model (zero acceleration).

![image](https://user-images.githubusercontent.com/23505408/209318082-f6019ccf-f3f6-4aaa-956a-cf0a8f3062b9.png)


#### High Process Noise & High Measurement Noise (Acceleration variance: 2 m/s2, Measurement variance: 3 m) 

The filter is still able to track the location well even with high measurement noise given that the process noise is high as well. The filter finds a compromise between the prediction (linear model) and the measurements (non-linear).

![image](https://user-images.githubusercontent.com/23505408/209318387-22b3dcb5-8e53-4e93-ab7d-83a10bb87331.png)

