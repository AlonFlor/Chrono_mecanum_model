# Chrono_mecanum_model

## Update Notes

* 08/16: `demo_IRR_mecanum_3`, 4-wheel version, only 



## Other Notes

Folder Structure: http://api.projectchrono.org/tutorial_install_project.html

### TODO

- [x] Specify front and back of the car. (Like take a screenshot of the car in the initial frame and label the wheel)
   ![](./img/Directions.PNG)

- [ ] Find a way to specify [v1, v2, v3, v4] to each wheel
   - [ ] Modify from code
   - [ ] GUI?
   	* If we got GUI, task 2 "randomly chosse velocity combination" could be done here, as what we have in PyBullet
   	* There is several GUI demo in the demos/. e.g. demo_IRR_tracks

- [ ] Find a way to generate trajectory. For example, output something like

	```
	frame	x	y	z	angle(quarternion/xyz angle)
	1		...
	```
- [ ] Visualize the dumped trajectory
- [ ] **Match ground truth data trajectory**


