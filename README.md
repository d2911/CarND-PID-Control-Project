# CarND-Controls-PID

Control the stearing angle of the car to make it stay in the center of lane using PID controller.

## Reflection

Cross Track Error (cte) is provided by simulator. On each raster, cte is used to determine the Proportional part, integral part and differential part of the error. Each part is multiplied with choosen parameter and summed up to find the total error.

In __UpdateError()__ function,
* differetial error part is calculated by subtracting current cte with previous cte.
* proportional error part depends only on current cte
* integral error part is calculated by summation of cte at each instance.

`void PID::UpdateError(double cte) {
   d_error = cte - p_error; 
   p_error = cte;
   i_error = i_error + cte;
}`

In __TotalError()__ funcrion, Each part is multiplied with choosen parameter and summed up to find the total error.

`double PID::TotalError() {
  double total_error = -(p_error * Kp) - (d_error * Kd) - (i_error * Ki);
  return total_error;  // TODO: Add your total error calc here!
}`

Kp, Ki and Kd values are choosen as 0.2, 0.0004 and 3.0 respectively. For car speed of 30mphr these values are preforming as expected.

`PID::PID() {
	Init(0.2,0.0004,3.0);
}`

Steaing angle to simulator should be in the rangle -1 to 1 and so calculate total error is limited.

`steer_value = std::min(std::max(steer_value_temp,-1.0),1.0);`

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

