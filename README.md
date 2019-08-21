# Sigma Technology Line Follower

![Front Image](images/front_image.jpg)
An ongoing project in creating an small robot that will follow a black line on a white background.

## Background
Hardware
## Installation
How to edit the code and push it

From the downloaded GitHub repo open the project in in Platform.io in Visual Studio Code. [Install Guide for VSC.](https://docs.platformio.org/en/latest/ide/vscode.html)


You will need to install the following libaries though Platform.io:
```
Libs enter here 1337
```
Flash the Arduino with the software provided.

In the software the pins for the servo and motor are defined as following:

```c
#define steeringControl 5
#define motorPin 4
```

## Usage
#### 1. Arduino
Boot up the Arduino with the server and motor connected to the corresponding pins.
#### 2. Jetson Nano
Open a terminal in the downloaded directory. 
Then run:
```bash
python ./start.py
```
The Jetson and Arudino should now have communication through the Ethernet cable and start working on controlling the car. If you have the Jetson Nano connect to an HDMI you can enable the output from the camera. To do that open the ```start.py``` file and look for 
```python
#Enable line below to show the final output on the screen
#cv2.imshow("result", combo_image)    
```
or 
```python
#Enable line below to show the black/white contrast output
#cv2.imshow("cropped", cropped_image)
```
Enable one or both options. First one will show you a result as the 1 output picture below and the second one will give you an output like picture 3 and 4.


## Example Images



## Contributers
[Martin Myhrman](https://skies.sigmatechnology.se/main.asp?rID=1&alt=2&username=miy)

