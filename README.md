# RC-car-controller---transmitter-and-receiver
This is to create a RC controller using the Arduino Nano, and also the NRF24L01 2.4GHz antenna.  Both the schematics for the transmitter and the receiver are in this repository.  I used a cheap car purchased at a dollar store, and it is set up to use a small servo motor.  


You will need to determine the pulse width of the center for your servo motor.  It also has a switch to calibrate the car to the tranmitter.

Use any tyoe of potentiometer to operate the x, and the y axis (x will be for steering, and y will be for throttle.)

I used joysticks with built in switch buttons that will be wired to two digital inputs on the transmitter, to turn on lights, and then one for strobes. 

The two buck converters are to operate the 5V to the servo motor, and also the 3.3 volt to the NRF24.  You will also need a 4 channel 5V to 3.3V digital converter for the signals between the Nano and the NRF as shown in the schematics

Your motor driver will be a L293.  If you need something bigger, than it can be substituted, as this is just a dual H-bridge
