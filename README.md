# Autonomous Drone Perception System

# Overview

This project develops a state estimation system for an autonomous drone using an Extended Kalman Filter (EKF), provides a C++ implementation of this system and performs adjustments of the system parameters in a flight simulator.

The system developed in this project is based upon a methodology by TELLEX, BROWN and LUPASHIN [1].

This project is part of Udacity's Autonomous Flight Engineer Nanodegree [2]. This README serves as a final report for the project.

<img src="videos/step_03_a_scenario_08.gif" height="400"/>
<img src="images/ekf_pseudo_code.jpg" height="400"/>

<br><br>





# Safety First!

1 - This project is educational only. The methodology used in this project was only used in the Udacity C++ flight simulator and not validated on a real drone.

2 - Only tune the parameters of your drone by following the procedure prescribed by your drone’s manufacturer.

3 - Make sure you comply with your local regulations before flying a drone.

4 - This project makes several assumptions which may not apply on a real drone.





# Install & Run

1 - Clone this repository

	$ mkdir -p /drone/projects/perception
	$ cd /drone/projects/perception
	$ git clone https://github.com/martin0004/drone_perception_system.git

2 - Install QTCreator and the GLUT libs

	$ sudo apt install qtcreator
	$ sudo apt install qtbase5-examples    # optional - qt examples
	$ sudo apt install qtbase5-doc-html    # optional - qt examples documentation
	$ sudo apt install freeglut3-dev

3 - Compile the project.

	$ cd /drone/projects/perception    # Make sure you open qt from here when compiling
	$ qtcreator                        # This will launch QTCreator

	File > Open File Or Project > simulator/project/CPPSim.pro
	Click on tab Edit on the left side panel of QTCreator
	Right click on CPPSim > Run qmake
	Right click on CPPSim > Run

4 - You should now see a drone hover. Right click on the simulation to switch between scenarios.

5 - Look into file `QuadEstimatorEKF.txt` to find all perception system parameters. If you change a value in this file and save it, the drone behavior will change in the simulation (no need to close the simulator, just update and save the file). Try to find better system parameters!





# [Optional] Udacity Starter Code

Udacity provided students with some starter code. This starter code can be installed with the following procedure.

	mkdir -p /drone/projects/perception/udacity_starter_code
	cd /drone/projects/perception/udacity_starter_code
	git clone https://github.com/udacity/FCND-Estimation-CPP





# Symbols and Units

| Units        | Description       |
|--------------|-------------------|
| m            | Distance.         |
| kg           | Mass.             |
| s            | Zoom.             |
| Hz           | Rate (frequency). |
| rad          | Angle.            |


| Acronyms | Description                         |
|----------|-------------------------------------|
| CF       | Complementary filter.               |
| EKF      | Extended Kalman filter.             |
| GPS      | Global positioning system.          |
| IMU      | Inertial measurement unit.          |
| wrt      | Abbreviation for “with respect to”. |


| Physical Constants | Description                 |
|--------------------|-----------------------------|
| g                  | Gravitational acceleration. |


| Indices          | Description                                   |
|------------------|-----------------------------------------------|
| $$x$$ (no index)   | True value of x (ground truth).               |
| $$x_{acc}$$        | Variable from IMU accelerometer measurement.  |
| $x_{GPS}$        | Variable from GPS measurement.                |
| $x_{gyro}$       | Variable from IMU gyroscope measurement.      |
| $x^b$            | Variable expressed in drone body frame.       |
| $\tilde{x}$      | Measured variable.                            |
| $\hat{x}$        | Estimated variable.                           |
| $\bar{x}$        | Predicted variable (intermediate EKF value).  |
| $x_t$            | Variable at time step t.                      |
| $x_{t-1}$        | Variable at time step t-1.                    |


| Measurement                   | Description                                     |
|-------------------------------|-------------------------------------------------|
| $\hat{x}$                     | GPS measurement - x location - global frame.    |
| $\hat{y}$                     | GPS measurement - y location - global frame.    |
| $\hat{z}$                     | GPS measurement - z location - global frame.    |
| $\hat{\dot{x}}$, $\hat{vx}$   | GPS measurement - x speed - global frame.       |
| $\hat{\dot{y}}$, $\hat{vy}$   | GPS measurement - y speed - global frame.       |
| $\hat{\dot{z}}$, $\hat{vz}$   | GPS measurement - z speed - global frame.       |
| $\tilde{\ddot{x}}^b$          | IMU measurement - x acceleration - body frame.  |
| $\tilde{\ddot{y}}^b$          | IMU measurement - y acceleration - body frame.  |
| $\tilde{\ddot{z}}^b$          | IMU measurement - z acceleration - body frame.  |
| $\tilde{p}$                   | IMU measurement - x body rate - body frame.     |
| $\tilde{q}$                   | IMU measurement - y body rate - body frame.     |
| $\tilde{r}$                   | IMU measurement - z body rate - body frame.     |
| $\tilde{\psi}$                | Magnetometer measurement - yaw - global frame.  |


| Errors                   | Description                                     |
|--------------------------|-------------------------------------------------|
| $\hat{e}_x$              | True error of variable or vector $\hat{x}$.     |
| $\hat{e}_d$              | Position magnitude error.                       |
| $\hat{e}_v$              | Velocity magnitude error.                       |
| $\hat{e}_{Euler}$        | Max Euler error.                                |


| State Variables          | Description                                     |
|--------------------------|-------------------------------------------------|
| $x$ (*)                  | State vector.                                   |
| $x$ (*)                  | Drone x position - global frame.                |
| $y$                      | Drone y position - global frame.                |
| $z$                      | Drone z position - global frame.                |
| $\dot{x}$, $vx$ (**)     | Drone x speed - global frame.                   |
| $\dot{y}$, $vy$ (**)     | Drone y speed - global frame.                   |
| $\dot{z}$, $vz$ (**)     | Drone z speed - global frame.                   |
| $\phi$                   | Drone attitude about x - global frame.          |
| $\theta$                 | Drone attitude about y - global frame.          |
| $\psi$                   | Drone attitude about z - global frame.          |

(*) Depending on the context, it will be obvious if x represents a vector or a position.
(**) Notation $vx$ will be used when $\dot{x}$ is difficult to read (e.g. when $vx$ is used as an index).

| Filters             | Description                                                     |
|---------------------|-----------------------------------------------------------------|
| $g$                 | EKF process model (called "transition model" in lectures)       |
| $g'$                | Function which derives the Jacobian of g.                       |
| $G$                 | Jacobian of g.                                                  |
| $h$                 | EKF measurement model.                                          |
| $h'$                | Function which derives the Jacobian of h.                       |
| $H$                 | Jacobian of h.                                                  |
| $Q$                 | Process noise covariance matrix.                                |
| $R_{bg}$            | Rotation matrix from body frame to world frame.                 |
| $R_{bg}[0:]$        | First line of $R_{bg}$.                                         |
| $R_{bg}'$           | Derivative of $R_{bg}$ wrt to yaw.                              |
| $R_{bg}'[0:]u[0:3]$ | Dot product of first line of $R_{bg}$ by first 3 elements of u. |
| $R_{GPS}$           | GPS measurement covariance matrix.                              |
| $R_{mag}$           | Magnetometer measurement covariance matrix.                     |
| $u$                 | Command vector.                                                 |
| $u[0:3]$            | First 3 elements of u.                                          |
| $x$                 | State vector.                                                   |
| $z$                 | Measurement vector.                                             |
| $T_s$               | Complementary filter sampling period.                           |
| $w_i$               | Complementary filter weight for sensor measurement i.           |
| $\bar{\Sigma}$      | EKF predicted state covariance matrix.                          |
| $\hat{\Sigma}$      | EKF estimated state covariance matrix.                          |
| $\nu$               | Noise probability distribution.                                 |
| $\sigma_x$          | Standard deviation of variable x.                               |
| $\tau$              | Complementary filter time constant.                             |



# Udacity C++ Flight Simulator

Parameters of the system developed in this project were tweaked by flying a drone in a series of scenarios in the Udacity C++ flight simulator [3]. The tuning scenarios are described in section “Validation”.

The flight simulator itself is a small QT application. The flying area is about 5 m x 5 m and the scenarios last only a few seconds. A contextual menu allows to switch between scenarios and  display charts of the drone state variables.

<img src="videos/simulator_-_overview.gif" width="700"/>

<br><br>

| Simulator Command        | Action               |
|--------------------------|----------------------|
| MOUSE LEFT               | Rotate               |
| MOUSE LEFT + X           | Pan                  |
| MOUSE LEFT + Z           | Zoom                 |
| MOUSE RIGHT              | Open contextual menu |
| UP                       | Apply force up.      |
| DOWN                     | Apply force down.    |
| LEFT                     | Apply force left.    |
| RIGHT                    | Apply force right.   |
| W                        | Apply force forward. |
| S                        | Apply force back.    |
| C                        | Clear graphs.        |
| R                        | Reset simulation.    |
| SPACE                    | Pause simulation.    |


# Vehicule

**Autonomy Architecture**

The autonomous drone in this project uses the classical perception, planning and control architecture.

This project focuses on developing the **perception** system. The **actuators**, **sensors** and **process** (i.e. the drone itself) are already implemented in the Udacity C++ simulator used in this project [3]. The **planning** and **control** systems are also part of the simulator. A series of predefined trajectories will feed waypoints to the control system during each simulation scenario. However in the last step of this project, the control system is be replaced by the control system which was developed in the 3rd project of this Nanodegree [4].

<br>
<img src="images/autonomy_architecture.jpg" width="700"/>
<br>
<br>

**Sensors**

| Sensor       | Measurement                 | Symbol                           | Rate (*) | Time Step (**) |
|--------------|-----------------------------|----------------------------------|----------|----------------|
| GPS          | x location - global frame   | $\tilde{x}$                      | 10 Hz    | 0.1 s          |
|              | y location - global frame   | $\tilde{y}$                      | 10 Hz    | 0.1 s          |
|              | z location - global frame   | $\tilde{z}$                      | 10 Hz    | 0.1 s          |
|              | x speed - global frame      | $\tilde{\dot{x}}$, $\tilde{vx}$  | 10 Hz    | 0.1 s          |
|              | y speed - global frame      | $\tilde{\dot{y}}$, $\tilde{vy}$  | 10 Hz    | 0.1 s          |
|              | z speed - global frame      | $\tilde{\dot{z}}$, $\tilde{vz}$  | 10 Hz    | 0.1 s          |
| IMU          | x acceleration - body frame | $\tilde{\ddot{x}}^b$             | 500 Hz   | 0.002 s        |
|              | y acceleration - body frame | $\tilde{\ddot{y}}^b$             | 500 Hz   | 0.002 s        |
|              | z acceleration - body frame | $\tilde{\ddot{z}}^b$             | 500 Hz   | 0.002 s        |
|              | x body rate - body frame    | $\tilde{p}$                      | 500 Hz   | 0.002 s        |
|              | y body rate - body frame    | $\tilde{q}$                      | 500 Hz   | 0.002 s        |
|              | z body rate - body frame    | $\tilde{r}$                      | 500 Hz   | 0.002 s        |
| Magnetometer | Yaw                         | $\tilde{\psi}$                   | 100 Hz   | 0.01 s         |

(*) Rate = 1 / Time Step.
(**) Cannot be faster than the controller time step (0.002 s).

<br><br>

# Coordinate Frames

<br>

**Body Frame, World Frame, Propeller Convention**

This project uses the same **body frame**, **world frame** and **propeller sign convention** as in the Drone Control System project of this nanodegree. See reference [4] for a descriptions of these frames.

<br>

**Rotation Matrix Rbg**

This project use rotation matrix Rbg, which is the rotation matrix from the body frame to the world frame [1]. Rbg is a function of Euler angles Ф (pitch), θ (roll), φ (yaw).

Euler angles Ф, θ, φ are provided as input to any estimator which needs to derive R internally.

<br>
<img src="images/rotation_matrix_-_fully_developped.jpg" width="700"/>

$\begin{matrix}
1 & 1 & 1 \\
1 & 1 & 1
\end{matrix}$
