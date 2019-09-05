# Sigma Technology Line Follower

<!---![Front Image](images/front_image.jpg)--->
An ongoing project in creating an small robot that will follow a black line on a white background.
  
## Hardware

###### Batteries
https://www.m.nu/batterier/lithium-ion-polymer-battery-37v-500mah
###### Battery charger
https://www.m.nu/dc-dc-lipo-laddare/usb-liion-lipoly-charger-v12
###### H-bridge
https://www.m.nu/servo-motorer-robotics/l298-dc-motor-driver-board
###### Arudino
https://www.funduinoshop.com/Funduino-MEGA2560-R3
###### Motors
http://www.hobbytronics.co.uk/mm-gear-motor-30hp-ext?keyword=Micro%20Metal%20Gearmotor%20HP  
https://www.tme.eu/se/details/pololu-989/tillbehor-till-mikromotorer/pololu/
###### Motor encoders
https://www.tme.eu/se/details/pololu-3081/tillbehor-till-mikromotorer/pololu/magnetic-encoder-pair-kit/
###### Wheels
https://www.tme.eu/se/details/pololu-1087/tillbehor-for-robotteknik-och-rc/pololu/pololu-wheel-32x7mm-pair-black/  
https://www.tme.eu/se/details/pololu-953/tillbehor-for-robotteknik-och-rc/pololu/ball-caster-with-1-2-metal-ball/
###### Sensor Array
https://www.tme.eu/se/details/pololu-4356/distanssensorer/pololu/qtrx-md-16rc-reflectance-sensor-array/







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
[Måns Lerjefors](https://skies.sigmatechnology.se/main.asp?rID=1&alt=2&username=lms)  
[Anton Lyngfelt](https://skies.sigmatechnology.se/main.asp?rID=1&alt=2&username=alt)  
[Martin Myhrman](https://skies.sigmatechnology.se/main.asp?rID=1&alt=2&username=miy)  
[Andrea Mastrorilli](https://skies.sigmatechnology.se/main.asp?rID=1&alt=2&username=dre)
[Rakshith Mukundu Rao](https://skies.sigmatechnology.se/main.asp?rID=1&alt=2&username=rmo)      
