# BUG-0-Algorithm-Dataset-Generator-(with animation)

Using random start and end points, this simulator generates datasets regarding a robot with a SICK sensor avoiding obstacles to reach the goal.

The most basic form of Bug algorithm (Bug 0) is as follows: The robot moves towards the goal until an obstacle is encountered. Follow a canonical direction (clockwise) until the robot reaches the location of initial encounter with the obstacle (in short, walking around the obstacle).

As a result of the SICK sensor, the robot receives information related to the environment in 180 values, which corresponds to the laser's 180 degree range. This information actually represents the distance being measured between the robot and the obstacle.

The angle 𝜃p shows the angle between the reference coordinate and the vector connecting the robot and the target. Also, 𝜃s displays the rotation angle of the robot.
In order to collect data, the robot was placed in an obstacle environment and according to the BUG0 algorithm, this information was collected in random possible positions of the starting and target points.

Any time, we store the distance of the obstacle (within 30 meters) in the index of each desired angle, based on the robot's head orientation.
<p align="center">
<img  width="439" alt="An example of traveling from the start point to the endpoint avoiding obstacles." src="https://user-images.githubusercontent.com/78047586/207312735-4a9d9c22-af70-4562-8005-7b6dab4f7fce.png">
</p>

-The variables you can alter before runnig the file:
  -show_animation = TRUE or FALSE
  -path_to_save_datas = "your path"
  -number_of_datasets = #The number of datasets you want to create

