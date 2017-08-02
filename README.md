#Fianl Project of Team 3

Dear User, 

Here are some notes about our navigation system. Hopefully, this can help you have a good start on our system.

##To Run the System 

The zip file contains two parts of our code : Pi and Arduino. Please run \"arduino.ino\" on Arduino first to initialize the vehicle and then run \"run\_with\_music.py\" on Pi to start the navigation system.


##About routes that cross buildings
If the route crosses buildings (like com1-2 and com2-2), please input 1 in the first step (whether the route crosses buildings). Otherwise, please input 0 (no crossing between buildings). After that, please input the name and the level of two buildings seperately, which is followed by the input of the start point and the end point.

If the route crosses buildings, the start point means the point's id in the first map and the end point means the point's id in the second map. For example, for com1-2 point 29 to com2-2 point 17, we need to input 29 and 17 seperately.

##Epilogue
We use the Festival library for our voice message. After having setting up the route, you can enjoy our navigation system by following the voice instructure !

Wish you have a good journey with our selected music.




Best regards,

Team 3 