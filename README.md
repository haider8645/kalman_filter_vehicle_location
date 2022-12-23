# kalman_filter_vehicle_location
This repository is the implementation of the Example 9 – vehicle location estimation at https://www.kalmanfilter.net/multiExamples.html

# Pre-reqs
 1. Eigen library
>    sudo apt-get install libeigen3-dev
 2. Matplot++ (https://alandefreitas.github.io/matplotplusplus/)
>    git clone https://github.com/alandefreitas/matplotplusplus/
>    cd matplotplusplus
>    mkdir build && cd build
>    cmake ..
>    make install

# Results
Try to play around with the process noise matrix Q and the measurement noise matrix R to see the different behaviors of the filter.
Additionally you can try different measurements to test the performance of filter.

1. Low Process Noise & Low Measurement Noise (Acceleration variance: 0.2 m/s2, Measurement variance: 1 m)

2. Low Process Noise & High Measurement Noise (Acceleration variance: 0.2 m/s2, Measurement variance: 3 m)

3. High Process Noise & Low Measurement Noise (Acceleration variance: 2 m/s2, Measurement variance: 1 m)

4. High Process Noise & High Measurement Noise (Acceleration variance: 2 m/s2, Measurement variance: 3 m)
