## Original paths

### original data 

* 2020114_CSG: GT CSG: `physics_based_robotics\real_robot_data\Jan14_2020_Control_Signals`
* 2020114_statefile: GT xytheta: `physics_based_robotics\camera_calibration\outputs\fixed_angle_dat_info`

### cut data (cut to ~360 frames) and corresponding pybullet matching data

* example_gt: Cutted ground truth data, approximately 360 frames (60 fps for 6 second) per file. They are cutted from `2020114_statefile`
* example_pybullet: PyBullet simulation data. If you generate exactly the same format in Chorno, you can use the tune2d to plot them together.



### code

* tune2d.py: Code used for matching: `physics_based_robotics\pybullet_robot\tune2D.py`
  * I already modify it to make it runable
  * In fact the data in `20200114_statefile` is not used.... The `state_files` variable in the for-loop is meaningless

