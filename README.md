## README
This is the final capstone project for ME449 Robot Manipulation class at
Northwestern University. The problem was to control and simulate a mobile manipulator where
the manipulator carries the box from the initial to the final configuration.
The software used here is MATLAB.

### How to use:
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
where there is an overshoot and third where we change the location of the box. \

The main script that runs all the functions together is final_project.m. Inside the script,
there are comments for the configuration changes for all the possible values. The main scripts
solve for a desired trajectory and then uses that as a reference to calculate an actual trajectory
from the current configuration of the robot. The software uses feedforward + Proportional
+Integral error correction for the current path to reach a zero steady state error, so that the robot
can perform the task with no error.