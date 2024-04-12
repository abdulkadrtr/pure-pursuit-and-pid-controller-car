## LEO DRIVE TASK SOLUTION
The vehicle follows the path using the **Pure Pursuit** algorithm. This algorithm first calculates the point in the `path_array` that is closest to the vehicle's position. Then, it selects the first point at a `look_head_distance` from the current position index as the target. The steering angle is then calculated based on the current vehicle orientation (yaw) and the angle between the two points.

The `look_head_distance` variable dynamically changes with speed. In other words, as the vehicle speeds up, the `look_head_distance` increases. The formula used is `look_head_distance = target_speed * K`, where `K` is a coefficient chosen as `K = 1.4`.

The vehicle's speed is regulated by the **PID** algorithm. The PID algorithm calculates the vehicle's acceleration based on the speed variable of the `target_point` found by the Pure Pursuit algorithm. The `P`, `I`, `D` coefficients were found experimentally and can be changed depending on further development options.

The `lateral_error` value is calculated based on the Euclidean distance between the point in the `path_array` closest to the vehicle's current position and the vehicle's instantaneous position. 

The `velocity_error` value is calculated based on the difference between `target_speed` and the vehicle's instantaneous speed.

A `config.xml` file has been created for the PlotJuggler tool. This file can be imported into the PlotJuggler interface to visualize the current errors.

## Demo Video

https://youtu.be/O1JgbL-Oa1c
