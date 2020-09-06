# STM-based Self-Driving Cart Controller 
[![License](https://img.shields.io/badge/License-BSD%203--Clause-orange.svg)](https://opensource.org/licenses/BSD-3-Clause)  

## 1. Introduction
Stm-cartcontroller is edge cloud based indoor self-driving cart.
This cart uses stm32f4discovery board.
It can drive itself with ultrasonic sensors, psd sensors and usb camera.
We can command to the cart with using edge cloud system.

## 2. Development project setup
* Step 1. Share a CubeMX file
* Step 2. Create "stm-cartcontroller" folder
* Step 3. Execute the shared CubeMX file
* Step 4. Save as CubeMX project (Project name: cart) in "stm-cartcontroller" folder
* Step 5. Click "Generate code" button
* Step 6. Delete main.h file in "cart/Inc/" and main.c file "cart/Src/"
* Step 7. Create/init local git repository in "stm-cartcontroller" folder
* Step 8. Set git origin "https://github.com/icns-distributed-cloud/stm-cartcontroller.git"
* Step 9. Git pull origin master

## 3. Project development
* Step 1. Rasie an issue and create a branch
* Step 2. Pull the branch to local repository
* Step 3. Checkout the branch
* Step 4. Develop functions, modules, or etc.
* Step 5. Git commit
* Step 6. Push the branch
* Step 7. Make "Pull request"

## 4. Result
* Obstacle avoidance
<img src="https://user-images.githubusercontent.com/53041199/92321743-654c0500-f067-11ea-8411-f52be2a7b3d8.gif">

* Demo Version
<img src="https://user-images.githubusercontent.com/53041199/92321739-5feeba80-f067-11ea-88a7-7bf8f653404b.gif">
