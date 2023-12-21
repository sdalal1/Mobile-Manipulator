## README
This is the final capstone project for ME449 Robot Manipulation class at
Northwestern University. The problem was to control and simulate a mobile manipulator where
the manipulator carries the box from the initial to the final configuration.
The software used here is MATLAB.

### How to use
<ol>
<li>NextState - The next state function takes in the current configuration of the robot and
returns an configuration after a timestamp </li>
<li>TrajectoryGeneratory - The trajectory generator function gives a desired trajectory of the
end effectory </li>
<li>FeedbackControl - The feedback control function is a controller which takes in the
current configuration and corrects it to give a better path </li>
</ol>

### Runs
There are three runs in this repository. First one where the error is minimized, second
where there is an overshoot and third where we change the location of the box.

The main script that runs all the functions together is final_project.m. Inside the script,
there are comments for the configuration changes for all the possible values. The main scripts
solve for a desired trajectory and then uses that as a reference to calculate an actual trajectory
from the current configuration of the robot. The software uses feedforward + Proportional
+Integral error correction for the current path to reach a zero steady state error, so that the robot
can perform the task with no error.

#### Best Run
The feedforward control plus PI parameters are as follows:
Kp = 0.7
Ki = 0.0001
Tsc_initial = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1]
Tsc_final = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1]

The graph for the best run:
<image src="https://github.com/sdalal1/Mobile-Manipulator/blob/main/results/best/best.png" title="Best run Graph"> </image>

The video of the best run is below:
<video src="https://github.com/sdalal1/Mobile-Manipulator/blob/main/results/best/best.mp4" title="Best Run" >
</video>

#### Overshoot
The feedforward control plus PI parameters are as follows:
Kp = 3.5
Ki = 0.5
Tsc_initial = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1]
Tsc_final = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1]

The graph for the overshooot run:
<image src="https://github.com/sdalal1/Mobile-Manipulator/blob/main/results/overshoot/overshoot.png" title="Best run Graph"> </image>

The video of the overshoot run is below:
<video src="https://github.com/sdalal1/Mobile-Manipulator/blob/main/results/overshoot/overshoot.mp4" title="Best Run" >
</video>

#### New Task
The feedforward control plus PI parameters are as follows:
Kp = 1.9
Ki = 0.001
Tsc_initial = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1]
Tsc_final = [0 1 0 1; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1]

The graph for the new position run:
<image src="https://github.com/sdalal1/Mobile-Manipulator/blob/main/results/new_task/New_task.png" title="Best run Graph"> </image>

The video of the new position run is below:
<video src="https://github.com/sdalal1/Mobile-Manipulator/blob/main/results/new_task/new_task.mp4" title="Best Run" >
</video>