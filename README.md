# traditional-CV-armorplate-tracker

Intro:
In robomaster game, I implement an aimbot system to help automatically obtain component robots' armerplate's 3D position under camera frame, 
sending stero information via Usart to a DJI A-type STM32 board, so that gimbal of our robot can move accordingly and shoot the target. 

Mechanism:
A traditional computer vision system that can track and draw bounding box on armplate with digits in real time.
Apart from tracking, the system can predict the position of armorplate center according to the motion of armorplate.

Algorithm:
solvePnp
pretrained resnet
Kalman Filter

Recommended IDE: 
Vscode

Camera Calibration:
Matlab

Hyperparameters:
process noise, measurement noise -> weightage of noise on prediction
gamma value, brightness -> relative and overall brightness of 

Setup: 
Readsene D435i camera 

Packages and tools required:
Realsense SDK
OpenCv
Fmt
Cmake toolchain

