# How to Use LEGO Mindstorms NXT with LabVIEW and myRIO
## Controlling LEGO Mindstorms NXT motors and sensors from a myRIO using LabVIEW

I will explain in this tutorial how to control LEGO Mindstorms NXT components from a myRIO running LabVIEW code.

I did this project a while ago during my studies and I thought it might interest some people. During an internship at the [University Transilvania of Brașov](http://old.unitbv.ro/en/Home.aspx) in Romania I had to work on a concept of robot for cleaning solar panels. I had at my disposal a myRIO running LabVIEW and LEGO Mindstorms NXT components. In order to make everything work together I had to "hack" those components to be able to control them from the myRIO.

This tutorial assumes you already have a basic knowledge on how LabVIEW works. It is divided in two parts:

#### 1. Hardware: The components specifications, modifications to make and PCB boards.

#### 2. Sofware: LabVIEW code running on the myRIO to command and/or read the components.

The components I'm going to talk about are:

* #### NXT motor with encoder
* #### NXT ultrasonic range sensor
* #### NXT touch sensor

## 1. Hardware
### myRIO

myRIO is a device developed by National Instruments. It’s an embedded hardware device, mostly destined to students for educational purpose. In order to teach engineering and to be used like a controller. As it's everything but cheap I don't expect this tutorial to reach hobbyist but could be useful to students.

<a href="url"><img src="/docs/images/picture1.jpg" align="center" height="555" width="555"></a>

The device can communicate through USB or Wi-Fi to a computer or other National Instruments devices (such as CompactRIO for exemple). For more information on the myRIO, the official data sheet can be found here: http://www.ni.com/pdf/manuals/376047c.pdf

Check it to get the pins layout.

#### Architecture

The device includes several inputs/outputs (analog and digital), two jack connectors (that act like input/output), ground connectors, 5V and 3.3V supply. myRIO also have an integrated 3-axis accelerometer and four programmable leds.

<a href="url"><img src="/docs/images/picture2.jpg" align="center" height="518" width="740"></a>

For the rest of the tutorial I would be referring to any digital input/output by "DIO" followed the number of the pin, and to any analog intput by "AI" followed by the number of the pin.

#### FPGA

myRIO includes a FPGA (Field Programmable Gate Array). Contrary to standard chips, the FPGA is a component which is physically modified to imitate logic gates in order to process the code. The main difference is the speed of execution (44MHz), superior to the real time processor, and the possibility to execute real parallel codes. However before every execution the program needs a long compilation time.

### LEGO NXT Wire

The Lego Mindstorm devices use a modified cable based on the RJ12 connector. The position of the latch is modified to make it incompatible with standard RJ12

<a href="url"><img src="/docs/images/picture3.jpg" align="center" height="223" width="298"></a> 
<a href="url"><img src="/docs/images/picture4.jpg" align="center" height="223" width="298"></a> 

*(Left picture) A normal RJ12 connector on the left, and an Lego NXT RJ12 connector on the right (source : www.philohome.com); (Right picture) Connector schematics*

In order to use the NXT devices, it’s important to be able to create those wires. A regular RJ12 sockets crimping tool can be used.
The other methods (which have been using), consists of cutting the latch and sticking it on the edge. This method is explained here:http://www.philohome.com/nxtplug/nxtplug.htm

The RJ12 connectors on the NXT control brick act like this:
* Pin 1 : PWM output signal 1
* Pin 2 : PWM output signal 2
* Pin 3 : Ground
* Pin 4 : 4.3V power supply
* Pin 5 : Input value (digital or analogic)
* Pin 6 : Input value (digital or analogic)

For each component I will detail the wiring of the RJ12 connector and the myRIO pins associate to it in my project (useful for the code and the PCB).

### NXT Motors

<a href="url"><img src="/docs/images/picture5.jpg" align="center" height="262" width="438"></a>
<a href="url"><img src="/docs/images/picture6.jpg" align="center" height="262" width="402"></a>

#### Wiring
The motors we will talk about are the ones included in the Lego NXT Mindstorm kit, often called "NXT Motor". Like the NXT sensors, the servomotor use a modified RJ12 cable, composed of 6 wires.
* Wire 1 – White : Motor supply 1
* Wire 2 – Black : Motor supply 2
* Wire 3 – Red : Ground
* Wire 4 – Green : Encoder supply(4.3V)
* Wire 5 – Yellow : Tach 1(Encoder 1 value)
* Wire 6 – Blue : Tach 2 (Encoder2 value)

The nominal power supply for the motor is 9V. Applying this voltage to one or the other motor supply allow to control the direction of rotation.

#### Mechanical Characteristics of the Motor
#### *No-load characteristics:*
* Nominal voltage : 9V
* Nominal speed : 170Rpm(Rotations per minute)
* Current : 60mA

#### *Characteristics when motor blocked (with 9V):*
* Torque to counter : 50 N.cm
* Current : 2A

Due to the high current, and even if the motor is protected by a thermistor, the motor will be damaged is the shaft is blocked for more than few seconds.

<a href="url"><img src="/docs/images/picture7.jpg" align="center" height="302" width="328"></a> 
<a href="url"><img src="/docs/images/image5.png" align="center" height="302" width="400"></a>

*(Left picture) rotation speed relative to applied voltage (from INFORMATION TECHNOLOGY AND DEVELOPMENT OF EDUCATION ITRO 2016) ; (Right picture) Speed and current in function of the load (from newmotorspot.co)*

#### PWM
In order to control the speed and the load of the motor, PWM (Pulse Width Modulation) technic is used. The power signal supplying the motor is a square signal alternating between high pulse and a 0V state. The width of the pulse, proportionally to the width of the whole period (call duty cycle) gives the percentage of the "voltage used by the motor". For example 9V supply with a duty cycle of 50% will act like a 4, 5V supply.

#### Rotary Encoders
A quadratic encoder is integrated in each motor. Two wires (one for each light sensor) give information about the actual position of the motor and are used to control it. This technology uses a disc rotating with the motor. A led illuminates through the holes of two tracks, and on the other side a photo sensor changes states when the encoder rotates.
<a href="url"><img src="/docs/images/picture8.jpg" align="center" height="256" width="297"></a> 
<a href="url"><img src="/docs/images/picture9.jpg" align="center" height="256" width="297"></a>

The pins used by the encoders are the following:
* Encoder Motor 1 : pin 23 and 25: DIO6 and DIO7
* Encoder Motor 2 : pin 27 and 29: DIO8 and DIO9

#### H-Bridge Board
Because myRIO can’t deliver 9V supply and handle directly the current needed for the motor, an electronic card with an H-bridge has to be crafted.

The H-bridge used in this tutorial is the famous L293.

<a href="url"><img src="/docs/images/image8.png" align="center" height="300" width="614"></a>

One H-bridge can control up to two motors. The pin ENABLE allows the motor to turn, and the two pins OUTPUT control the direction.

<a href="url"><img src="/docs/images/image9.png" align="center" height="200" width="408"></a>

By applying a PWM to one of the input, and by supplying the enable and the other input we allow the motor speed to be controlled. To invert the direction, the other input need to be put down.

Applying the PWM on the enable is less effective. Indeed, the motor will act half powered (when pulse is high) and totally not-powered (pulse down). This reduces the load of the motor.

The card is based on the schematic of the Motor Driver 2A Dual L298 H-Bridge for Arduino. Each board can handle two motors. Four leds allow the user to know if the motor is powered supplied and for which direction. Schematics and files can be found in the git project folder.

<a href="url"><img src="/docs/images/picture10.jpg" align="center" height="445" width="615"></a> 

The board is made to be directly connected to the myRIO outputs. A connector on the board allows connecting other devices to the myRIO through the board (because the board takes all the space for the outputs on the myRIO).

The pins used for motor power supply and H-bridge control are listed below (9V) :
* Input 1 : pin 13 : DIO1
* Input 2 : pin 15 : DIO2
* Enable A : pin 11 : DIO0
* Enable B : pin 17 : DIO3
* Input 3 : pin 19 DIO4
* Input 4 : pin 21 DIO5

In order to control 4 motors in my project (2 NXT motors for driving, a NXT motor for controlling the arm angle and a standard DC motor to rotate a brush) I needed 2 PCBs (as each one can command 2 motors). I won't explain how to make a PCB as internet is full of tutorials on how to do it. Special thanks to Prof Dr. Petru Cotfas for the PCB design.

#### Current Sensing

You can measure the current going through the DC motor by reading the SNS0 and SNS1 pins. On each channel will be a voltage proportional to the measured current, which can be read as a normal analog input, on the analog inputs A0 and A1. It is calibrated to be 3.3V when the channel is delivering its maximum possible current that is 2A.

In this project the pins for motor sensing are:
* Current sensing Motor 1: AI0
* Current sensing Motor 2: AI1

### NXT Ultrasonic Sensor

<a href="url"><img src="/docs/images/picture11.jpg" align="center" height="500" width="500"></a>

As its name suggest, an ultrasonic sensor use ultrasound to measure the distance to an obstacle. The principle is very simple: an ultrasonic wave is sent by the sensor, and will collide on every object in front of it. The wave will then come back to the sensor, and the time between the sending and the receiving is proportional to the distance to the object. The drawbacks of this technology are the inability to measure very close object (there is a “dead zone”) and the impossibility to use multiple ultrasonic sensors in the same time (they will disturb each other).

#### Wiring
* Wire 1 – White : +9V powersupply 
* Wire 2 – Black : Ground
* Wire 3 – Red : Ground
* Wire 4 – Green : +4.3V powersupply
* Wire 5 – Yellow : I2C Clockline (SCL)
* Wire 6 – Blue : I2C Data line(SDA)

#### Characteristics

Here the characteristics of the sensor:
* Range : From 0 to 255 cm (theory range, real range is from 4cm to 255cm due to the “dead zone”)
* Accuracy : +/- 3cm
* Directivity : Cone of approx.30°

The sensor is more efficient with hard and large surface objects. Soft and curved objects give a worse reading.

The sensor can be used in different mode: single shot or continuous mode. By default the sensor is in continuous mode, and it’s continuously making new measurements that are registered on the first memory slot. In single shot mode, new measurements are made when asked, and stored in every memory slots (8 available, which means it’s possible to store maximum 8 values). For this application, we only use continuous reading mode.

#### Accuracy Test

Those tests are not really necessary to use the sensor but I thought they would be a nice addition to the topic. (Test not made by me, data are from http://www.tik.ee.ethz.ch/mindstorms/sa_nxt/index.php?page=tests_us).

#### *Static Test*

Test made with a static sensor, the object used as a target is a small cardboard box (14.5 cm x 9.5 cm x 6 cm).

<a href="url"><img src="/docs/images/image12.png" align="center" height="393" width="614"></a>

From the results above it can be deducted (for a static use), that for small distance the accuracy is good (for a non-industrial sensor) and not very accurate at the very beginning of the range (measured value bigger than real value).

#### *Dynamic Test*

Test made by moving slowly the sensor to a wall in continuous reading mode (speed of approximately 30cm/s).

<a href="url"><img src="/docs/images/image13.png" align="center" height="393" width="614"></a>

From the results above, it can be deducted that some areas are not accurate, especially between 25cm and 50cm where the sensor can return the value of 48cm. Also on all the range the sensor returns sometime the error value (255).

#### Communication

The sensor uses the I2C protocol to communicate. This is a protocol Slave/Master that uses 2 wires : one for clock and one for data. The wire for data can be reversed to transmit data from the master or from the slave during the communication.

In this project, the pins used for the communication are:
* SCL (Clock) : DIO12 (ConnectorMXP B)
* SDA (Data) : DIO13 (ConnectorMXP B)

The address of the device is 02 in a 8-bit context or 01 in a 7-bit context.

Also the I2C protocol used by NXT is a little bit different than standard I2C. First the frequency is not the same as standard speed nor high speed I2C; it’s around 9600hz (measured value around 11khz). Furthermore to communicate with this sensor you will need to add an extra pulse without data after any Repeated Start.

#### Mistakes to Avoid

One of the problem by using myRIO was that some digital Input/Output from myRIO are using a pull-up resistor of low resistance (the DIO 14 and 15 from the connector A and B). The sensor wasn’t able to pull down the line enough to transmit data, it’s important to use other DIO, which are using a bigger resistor.

<a href="url"><img src="/docs/images/image13.png" align="center" height="348" width="587"></a>
<a href="url"><img src="/docs/images/image14.png" align="center" height="348" width="587"></a>
<a href="url"><img src="/docs/images/image15.png" align="center" height="348" width="587"></a>

Also the sensor need 4V for its I2C communication and myRIO delivers only 3.3V. It’s enough to make the communication works, but it’s also possible to use a board that allowsI2C between devices with different voltages.

#### Information Delivered

The device returns a value: a 8 bits integer directly proportional to the distance (the value in decimal is the distance in centimeters). Except for the value 255, which means nothing in sight (out of range) or an error.

### NXT Touch Sensor

The touch sensor is a very basic sensor. It’s a digital sensor that is either on high or low state.

<a href="url"><img src="/docs/images/image16.png" align="center" height="450" width="600"></a>

The wiring is as follow:
* Black : ground
* Green : 4.3V supply
* White : output

When high (4, 9 V) sensor is not activated
When low (<1 V) sensor is activated

## 2. Software

The myRIO is the robot controller and is programmed using LabVIEW. Its program is split into two parts: a part of the code running on the FPGA (for the advantages and drawbacks see FPGA section above), and the other part of the code running in the Real-Time controller, a "standard" processor.

### FPGA Code Overview

The code on the FPGA mostly consists of a while loop for each actuator/sensor.

<a href="url"><img src="/docs/images/image17.png" align="center" height="405" width="960"></a>

### Motor Control

#### PWM

The motor control is done by using PWM (Pulse Width Modulation as explained above). The four motors are controlled in the same loop in order to reduce the space taken on the FPGA.

<a href="url"><img src="/docs/images/image18.png" align="center" height="464" width="831"></a>

#### *NXT Motors*

For the first three motors (NXT Motors described above), the code is the same. The speed value, direction and motor state (enabled or disabled) are sent by the real-time host

The "enable" value is directly connected to the Enable pin. For the direction, a case structure turns the value of one of the output pin on high or low level. The polarity of the other output pin is managed in a sequenced structure.

<a href="url"><img src="/docs/images/image19.png" align="center" height="203" width="960"></a>

* (1) The first frame initializes a global timer for the loop (in this case 20 000 ticks), that means the whole sequence will always last this long.
* (2) The second frame deactivates the other motor output. In the same frame a calculation is done: the whole loop time minus the time sent by the real-time host (the -1 decremental function used on the result is to take in account the time spent in this frame, 1 tick).
* (3) Third frame: wait function using the result of the calculation. The output stays inactive during this time.
* (4) Last frame: After the wait, the output is activated; this frame stays until the global timer is elapsed.

So the speed of the motor is modulated by the time sent by the real-time host. Indeed, this time will change how wide the high pulse is, proportionally to the period time.

#### *DC Motor (no NXT)*

The fourth motor is not a Lego NXT motor, it’s a standard DC motor used to make the brush of the robot turns in my project. This motor needs a 6V power supply and only a 9V is available for my project. In order to reduce the voltage a fixed value PWM is used. The other output is wired on true and the enable is the only controllable parameter. This part is not necessarily useful if you don't plan to add an external motor to your project but I found that it was a nice addition to the tutorial.

<a href="url"><img src="/docs/images/image20.png" align="center" height="263" width="960"></a>

#### PID

The value delivered to the FPGA motor control loop is calculated in a sub-vi in the real-time host. A PID (Proportional Integral Derivative) function is used. If you don't know what a PID is you might want to google it before continuing.

<a href="url"><img src="/docs/images/image21.png" align="center" height="429" width="960"></a>

<a href="url"><img src="/docs/images/image22.png" align="center" height="418" width="949"></a>

<a href="url"><img src="/docs/images/image23.png" align="center" height="636" width="620"></a>

Here the explanation for the code (see pictures above)
* (1) PID Values sent by the user :
* *PID gains, which needs to be determined*
* *Reset (to reset the output)*
* *Velocity wanted* 
* *Actual velocity, measured by the encoders (in the case of a speed PID, it will be the position in case of a position PID)*
* (2) PID output (calculated through the function), vary from -100 to 100
* (3) Test if the result is negative (move backward), and modifies the value of “direction”
* (4) Calculations:
* *Absolute value 
* *Multiplies by the period (20000 ticks)
* *Divides by 100 to obtain a result proportional to the period
* (5) Current sensing : if the value is superior to 2A, which is read as the value “2702”through the analog input, the Boolean value become true.

#### Encoders

The encoder included in the NXT motor is a quadrature encoder, a type of incremental encoder. As explain in the hardware part, this sensor has two digital channels.

<a href="url"><img src="/docs/images/image24.png" align="center" height="466" width="766"></a>

* If the value of the channel B is the same as the previous value of the channel A, the counter is moving forward.
* If the value of the channel B is not the same as the previous value of the channel A, the counter is moving backward.

The program for the encoder is split into two parts to save space on the FPGA. The role of one of the program part (running on FPGA) is to acquire data and the role of the other (running on the real-time host) is to compute the results.

#### *FPGA*

As written above the FPGA part of the encoder reading code focus on acquiring data. The three encoders are put in the same loop.

<a href="url"><img src="/docs/images/image25.png" align="center" height="620" width="613"></a> 
<a href="url"><img src="/docs/images/image26.png" align="center" height="319" width="960"></a> 

* (1) Compare, for each channel, the current value with the previous value. If for one channel the value changes, it means there is a movement and the Boolean value changes to notify it.
* (2) Knowing if there is movement or not, the program chooses either to add the value to the position shift register or to use the previous position value.
* (3) Compare (as explain before) the previous value of the channel A with the current value of channel B to know if it’s an increment or a decrement.
* (4) Reset button that put the 0 value in the position shift register

#### *Real time*

The encoder code on the real time controller computes the data (position) obtained in the FPGA code to get the angle, the speed and the acceleration.

<a href="url"><img src="/docs/images/image27.png" align="center" height="596" width="916"></a> 

<a href="url"><img src="/docs/images/image28.png" align="center" height="438" width="880"></a>

<a href="url"><img src="/docs/images/image29.png" align="center" height="618" width="960"></a>

<a href="url"><img src="/docs/images/image30.png" align="center" height="460" width="535"></a>

First, the code uses the position to know the angle (absolute value), number of tours, and the angle in the in the current tour.
* (1) Divides the number by 4 of count to have the number of pulses.
* (2) Divides by 180 to obtain the number of turn (a pulse each two degrees). The rest is used (after being multiplied by 2) to have the value of the degree in the current tour.
* (3) Multiplies the number of tour by 360 and add the degree of the current tour to have the number of degree in absolute.

The code also computes the velocity and the acceleration. To do that a “Time Interval” is chosen by the user. The program will then count the number of pulses during this interval (Pulses/interval = speed)

The calculation of the acceleration just use the shift register to know the difference between current speed and the last iteration speed.

For the initialization, a case structure with a first call function helps to initialize the value.

### Ultrasonic Sensor

As for the motor, the control of the ultrasonic sensor is split between the FPGA and the real-time host.

#### *Real-Time Sub-Vi*

<a href="url"><img src="/docs/images/image31.png" align="center" height="568" width="960"></a>

This simple vi mostly consist of a case structure. This case (empty for the value “false”) will be executed only if the FPGA sends the value “I2C FPGA Running” as false.
* (1) Inside the case a sequence structure will first read the previous value delivered by the FPGA code (and then convert it into a number)
* (2) Waits 10ms
* (3) Sends again value to be sent through I2C protocol to the sensor

/!\ Be careful about the value “255”, which is a “nothing in sight” value (or error) and that can be delivered by mistake when the sensor is moving (see accuracy tests in the hardware part of the tutorial). It is recommended to collect the distance a few times before giving an order to the robot (or to ignore 255 values)

#### *FPGA Part*

As explained before, the sensor communicates with I2C. However I was unable to connect to the sensor using the existing I2C function in LabVIEW. I realized by analyzing the signal with an oscilloscope that the I2C protocol used by LEGO presents some differences with the standard protocol.

* Firstly, the frequency of the communication is not the same as standard speed nor high speed I2C, but around 9600hz according to data sheet. However by analyzing the signal between the sensor and the control brick, we get a value around 11khz. This value is used in the code.
* Every time you need to read a value from the sensor, a repeated start is needed. Even if repeated starts exist in standard I2C, they are not very common.
* It’s also needed to **add an extra pulse without data after any Repeated Start.**

<a href="url"><img src="/docs/images/image32.png" align="center" height="266" width="960"></a>

The communication is made by a sequence structure. This sequence reads the first register, that contains the value of the continuously reading mode (no other modes are used).

According to the sensor data sheet, the format of the signal sent is:

<a href="url"><img src="/docs/images/image33.png" align="center" height="120" width="615"></a>

The three first frame initialize the communication (there are making the “Start Condition” of every I2C communication: both data and clock are up, data goes down and then clock goes down). A boolean variable “I2C FPGA Running” is set to true to prevent the real-time host from acting until the communication is over.

<a href="url"><img src="/docs/images/image34.png" align="center" height="336" width="615"></a>

Then, the first and second bytes are sent through a “Write” sub-vi.

<a href="url"><img src="/docs/images/image35.png" align="center" height="529" width="615"></a>
 
This sub-vi is composed of three frames. The first sends the data through a “1-byte transfer” sub-vi, which we will detail below. Then another sub-vi manages the acknowledgement. The last frame goal is to check if the line is held low by the receiver to finish.

<a href="url"><img src="/docs/images/image36.png" align="center" height="359" width="960"></a>

Inside the “1-byte transfer” sub-vi, the “1-byte transfer” sub-vi is running in a while loop executed 8 times (8bit = 1byte). In this loop another sub-vi sets the value of the data line to either 1 or 0 (in function of the bit wanted) and then made one clock pulse.

<a href="url"><img src="/docs/images/image37.png" align="center" height="266" width="623"></a>

The "acknowledge" sub-vi uses a “Set Output Enable” function to change the behavior of the output/input for data. The Boolean value “Acknowledgement” is used to know if the acknowledgement is sent by the master or the slave. While doing this, a clock pulse is made.

<a href="url"><img src="/docs/images/image38.png" align="center" height="415" width="615"></a>

An empty 1-bit transfer function is used to simulate an extra pulse without data. The repeated start is obtained very easily by a simple sequenced structure.

<a href="url"><img src="/docs/images/image39.png" align="center" height="488" width="615"></a>

Right after the repeated start, the third byte is sent through the same “Write” sub-vi as before. A delay is applied is the next frame. This delay tends to imitate the behavior of the NXT brick when it communicates with the sensor (seen by analysis).

It’s better to put this delay in order to give the sensor some time to react. The last frame uses an “I2C Read” to read the value from the device.

This sub-vi works the same way as the “Write” sub-vi. Except that the value in read instead of written. 

<a href="url"><img src="/docs/images/image40.png" align="center" height="153" width="623"></a>

A “Set Output Enable” function is used to turn the pin into an input

<a href="url"><img src="/docs/images/image41.png" align="center" height="293" width="387"></a>

And finally the stop condition is sent (clock high, then data high). In the meantime the values “I2C Start” and “I2C FPGA Running” are set to low again.

### Touch sensor

For the touch sensor, the use of only one function in the real-time host is enough. The sensor is just connected to a digital input.

And finally the stop condition is sent (clock high, then data high). In the meantime the values “I2C Start” and “I2C FPGA Running” are set to low again.
Touch sensor

<a href="url"><img src="/docs/images/image42.png" align="center" height="216" width="438"></a> 438px × 216px

## Conclusion

Now you get everything you need to know in order to make the LabVIEW code works on myRIO! I hope this project/tutorial will help some people.

## Authors

* **Robin Baran** - *Initial work*

## Acknowledgments

* Prof Dr. Petru Cotfas for welcoming me in his lab and helping me when needed.
* PCB Design by Dr. Petru Cotfas
* How to create RJ12 NXTconnector, http://www.philohome.com/nxtplug/nxtplug.htm
* L293 datasheet and use, https://www.sparkfun.com/products/9670
* H-brigde driver board, http://arduino.cc/en/Main/ArduinoMotorShieldR3
* Arduino and NXT Sensors,https://sites.google.com/site/mccolganrobotics/
* I2C communication with theulstrasonic sensor, http://www.picaxe.com/Circuit-Creator/Sensors/Lego-NXT-Ultrasonic/
* Direct manipulation of I2C pins,https://developer.mbed.org/forum/mbed/topic/804/
* Direct manipulation ofI2C pins – ultrasonic sensor http://www.tik.ee.ethz.ch/mindstorms/sa_nxt/index.php?page=print
* Interface I2CPort for Lego NXT,http://www.lejos.org/nxt/nxj/api/lejos/nxt/I2CPort.html#i2cStatus()
* LEGO MINDSTORM NXT HardwareDeveloper Kit, http://www.legolab.daimi.au.dk/DigitalControl.dir/LEGO%20MINDSTORMS%20NXT%20Hardware%20Developer%20Kit/LEGO%20MINDSTORMS%20NXT%20Hardware%20Developer%20Kit.pdf
* National Instruments website, www.ni.com
