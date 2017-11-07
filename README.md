# **Extended Kalman Filter Project**

## **1.compile**
The code compiled without errors. The result is in "[build](./build)" folder.

## **2.Accuracy**
The RMSE = [0.0961, 0.0922, 0.4525, 0.4435]  
[Here](./video/out.ogv) is the vido of my monitor.

## **3.The follows of my algorithm**
### *3.1 I use the first measurements to initialize the "x_".*  

If the first measurement comes from RADAR my code shows in below.    
```c++
  float rou = measurement_pack.raw_measurements_[0];
  float phi = measurement_pack.raw_measurements_[1];

  ekf_.x_ << rou*cos(phi), rou*sin(phi), 0, 0;

  previous_timestamp_ = measurement_pack.timestamp_;

```
If the first measurement comes from LASER my code shows in below.
```c++
  ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],0,0;

  previous_timestamp_ = measurement_pack.timestamp_;
```
### *3.2 After the first measurement I make a predicion use the 'Predict' method of the `kalman_filter` class.* 

 Here is the steps:
 - calculate the elapsed time between the current and previous measurement.
 - Modify the `F` matrix and `Q` matrix.
 - Call the `Predict` method of the `kalman_filter` class.

### *3.3 After prediction I make a update.*
According to the type of measurement I use different update method.  
For LASER I use the `Update` method of the `kalman_filter` class. For RADAR data I use the `UpdateEKF` of the `kalman_filter` class.  

There are some differnt in `Update` and `UpdateEKF`:  
 - The `R` matrix is different
 - In `Update` the `Z_pred` is calculated with the `H` matrix, but in `UpdateEKD` the `Z_pred` is calculated with `h(x)` function
 - In `Update` the `S`,`K`,`P` matrix is calculated with `H` matrix, but in `UpdateEKF` they are calculated with `Hj`(defined in `Tools` class) matrix.


