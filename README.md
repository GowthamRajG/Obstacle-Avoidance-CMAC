# Obstacle-Avoidance-CMAC

The project is to add obstacle avoidance feature to a Pioneer 3DX Robot in the V-REP environment(with active keyboard control). The code was written in Python and interfaced with V-REP through a set of API functions that control the simulation environment.

Pioneer 3DX is a compact differential-drive mobile robot. The sensor system includes 8 front-facing ultrasonic sensors and a maximum of 3 swappable batteries. In default, the sensor readings from all the front-facing sensors are squared up in an array to detect the obstacle and actuate the motor to avoid them.

I have implemented a back-propagation algorithm to learn the sensor readings. The algorithm would take the readings of front-facing ultrasonic sensors and square them up. A weight vector has been initialized that would hold the weightage for the input values. The weight vectors are assigned a value that is like a continuous cell CMAC architecture. The sensor readings are then trained for a fixed number of cycles (say 10, 100) and the trained value is re-assigned to the sensor-output value. This value is then used to actuate the motor values for left and right wheels. The learning cycle was quick and the robot has avoided the obstacle from the distance specified.

The keyboard control has been implemented through input() function of Python. Keyboard strokes ‘1’ is used to steer the vehicle and stroke ‘2’ is used to stop the vehicle (temporarily disabled for finishing the task but can be un-commented at any time).

The robot can wander while actively avoiding the collisions.

