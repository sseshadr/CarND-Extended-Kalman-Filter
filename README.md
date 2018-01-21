# CarND-Extended-Kalman-Filter
Use Kalman filter principles to fuse LiDAR and RADAR sensor data to provide a more accurate estimate of a simulated vehicle's position and velocity.

#### NOTE: This submission depends on the Udacity visualization and data generation utilities from the utilities repository [here](https://github.com/udacity/CarND-Mercedes-SF-Utilities).

---

[//]: # (Image References)

[image1]: ./readme_images/measFcn.PNG "Measurement Function"

[image2]: ./readme_images/radarLaser.PNG "Sensor Fusion Result"

[image3]: ./readme_images/laserOnly.PNG "Laser Only Tracking"

[image4]: ./readme_images/radarOnly.PNG "Radar Only Tracking"

---

#### Project Notes

##### Initializing the State Vector
The state vector position is initialized using the radar/lidar sensor measurements. The velocity states are an initial guess. This guess has an effect on the overall accuracy of the filter. We can tune this to get the filter to robustly perform for different datasets.

##### Calculating y = z - H * x'
For lidar measurements, the error equation is y = z - H * x'. For radar measurements, the functions that map the x vector [px, py, vx, vy] to polar coordinates are non-linear. Instead of using H to calculate y = z - H * x', for radar measurements we will use the equations that map from cartesian to polar coordinates: y = z - h(x'). Here h(x) is as follows:

![alt text][image1]

##### Normalizing Angles
In C++, atan2() returns values between -pi and pi. When calculating phi in y = z - h(x) for radar measurements, the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. The Kalman filter is expecting small angle values between the range -pi and pi. We can do this using: angle = atan2(sin(angle), cos(angle)).

##### Avoid Divide by Zero throughout the Implementation
We can write a simple check to see if denominators don't result in 0. This is built into the jacobian calculation.

##### Test Your Implementation
Testing this implementation on both datasets 1 and 2, we are able to pass the rubric check. The px, py, vx, and vy RMSE is less than or equal to the values [.11, .11, 0.52, 0.52].

![alt text][image2]

We can also see that sensor fusion from the 2 sensors returns a better accuracy of the estimate compared to either of the 2 sensors alone as shown below.

Laser only:

![alt text][image3]

Radar only:

![alt text][image4]
