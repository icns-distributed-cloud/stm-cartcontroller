# STM-based Self-Driving Cart Controller 
[![License](https://img.shields.io/badge/License-BSD%203--Clause-orange.svg)](https://opensource.org/licenses/BSD-3-Clause)  

## Introduction
**'stm-cartcontroller' is essential to edge cloud based indoor self-driving cart.**
The controller is running on stm32f4discovery board.
The cart can sense surroundings by ultrasonic sensors, psd sensors and usb camera.
A Moving Edge Cloud in the cart and Fixed Edge Clouds communicate. The cart can drive autonomously.

## Getting started with stm-cartcontroller
## 1. Development project setup
* Step 1. Share a CubeMX file
* Step 2. Create "stm-cartcontroller" folder
* Step 3. Execute the shared CubeMX file
* Step 4. Save as CubeMX project (Project name: cart) in "stm-cartcontroller" folder
* Step 5. Click "Generate code" button
* Step 6. Delete main.h file in "cart/Inc/" and main.c file "cart/Src/"
* Step 7. Create/init local git repository in "stm-cartcontroller" folder
* Step 8. Set git origin "https://github.com/icns-distributed-cloud/stm-cartcontroller.git"
* Step 9. Git pull origin master

## 2. Project development
* Step 1. Rasie an issue and create a branch
* Step 2. Pull the branch to local repository
* Step 3. Checkout the branch
* Step 4. Develop functions, modules, or etc.
* Step 5. Git commit
* Step 6. Push the branch
* Step 7. Make "Pull request"

## Result demo
### Obstacle avoidance demo
When a object is suddenly dropped in front of the cart, cart avoid the obtacle and drive safely to the destination. :smile:
<img src="https://user-images.githubusercontent.com/53041199/92321743-654c0500-f067-11ea-8411-f52be2a7b3d8.gif">

### Indoor self-driving demo
The cart drives automonusly and safely by the self-controll system and communication between edge clouds. :thumbsup:
<img src="https://user-images.githubusercontent.com/53041199/92321739-5feeba80-f067-11ea-88a7-7bf8f653404b.gif">
