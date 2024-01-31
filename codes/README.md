In this folder, I present two codes used in some microcontrollers in the vehicle.

The first one, the Steering.ino, uses an ESP32 to control the Steering subsystem. The main logic of the code is that if a CAN message is received, then the setpoint is changed, and with the setpoint and the reading of a Time of Flight Sensor, the error is computed. Finally, with the error, a simple ON-OFF control is implemented, and the steering motor is moved.

The second one, the ESP32_PIDVelocityV2.ino, is in charge of controlling the speed of the vehicle. 
The main logic is that if a CAN message is received, then a new setpoint is saved. At the same time, a pulse counter is implemented for reading a hall sensor in the power train of the vehicle to compute the RPM and the speed. Finally, with these values, the error can be computed, and after making the tunning of the PID values for the control, the PWM signal can be sent to the power train controller.
