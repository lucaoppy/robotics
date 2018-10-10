# Robotics Project 2015/2016

Developed by
- Lazzaretti Simone 807019
- Oppedisano Luca 806628

Appello date: 27/06/2016
Delivery date: 10/08/2016

## Introduction
We worked on Ubuntu 15.04 with Gazebo 7.0.0

We managed to build our SDF model and the plugins to control speed, camera and integration.

We had some technical problems with the installation of `ros-jade-gazebo7-ros-pkgs` (Unable to locate package). Even if we had all the repositories and keys set we were not able to retrieve that package, so we could not make the connection between ROS and Gazebo. We also tried different procedures and reinstalled Linux from scratch but the same error occurred. For this reason we could not start part 4 and 5 of the project (navigation, planning and monitoring).

## Prepare the environment
- Download the .zip file attached to the e-mail
- Extract it and put the "kobra2" folder where gazebo can find it
	e.g `/home/USERNAME/.gazebo/models/kobra2/`

- Open a terminal and go into `kobra2/plugins/build`

- Then run `cmake ../ && make` to compile our plugins
	(in `kobra2/plugins/build/` the file "libmodel_push.so" and others will be created)

- Inside the "kobra2" folder open the file "model.sdf" with any text editor

- At line $5$ set the "filename" attribute's value to the location of "libmodel_push.so" file
	e.g. `filename="/home/USERNAME/.gazebo/models/kobra2/plugins/build/libmodel_push.so"`
- Save everything, open a terminal and run `gazebo --verbose`

## Run the simulation


- From the "Insert" tab of Gazebo insert a "Kobra2" model
- You will now see some info in the terminal such as:
	- Time elapsed from previous measurement
	- Linear speed of the robot (central average)
	- Linear speed of each wheel with an ASCII model
	- Angular speed of the robot
	- Theta (trajectory angle with repect to y axis) 
	- Angular speed of each wheel with an ASCII model
	- Integration type (default Euler)
	- Displacement on X and Y axes

- Open another terminal and go into `kobra2/plugins/build`
- From here you can run:
	-	`./vel a b` with:
		-	a = desired linear speed (m/s)
		-	b = desired angular speed (rad/s)
	-	`./int c` with:
		-	c = $\begin{cases}
		1\text{ for Runge-Kutta mode}\\
		2\text{ for Exact mode}\\
		\text{any other number for Euler mode}
	\end{cases}$
	-	`./cam d e` with:
		-	d = tilt angle in degrees (0 is the initial position)
		-	e = pan angle in degrees (0 is the initial position)
- You will see the change of behaviour in the first terminal or in Gazebo or in both
- You can run the commands in any sequence at any time
- You can remove the robot from Gazebo and place another Kobra2 whenever you want


## Known Issues - Expected Behaviours
- Our robot is built to move along y axis instead of x axis initially (visual reasons)
- Time between two measurements will always be over 100ms (tweaked with a sort of timer to avoid Real Time Factor drop)
- If you choose Exact integration it will be used only when possible, in other cases Euler algorithm will run (avoid division by 0 when angular speed is 0)
- Integration errors occur due to approximations in the code
- Setting the camera position to something different from [0, 0] causes a visual vibration of the chain that somehow interferes with the robot movements, you can limit this effect by setting small angles and keeping the robot moving. Notice that the camera visual in `Ctrl+T, gazebo.msgs.ImageStamped`  does not vibrate
