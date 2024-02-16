# Session report 4:

**Session objective:** Test the algorithm I wrote during the previous session, fix bugs, and implement new functionalities.

## Issue with serial speedrate

First of all, I had a significant issue with the code's execution speed. The algorithm slowed down because of the communication speed, which was only 9600 bits per second. The Arduino couldn't sustain the motor pins at high levels while printing the number of ticks from the encoder on the serial connection at this speed. I resolved this by initializing the communication speed to 115200 bits in the Serial.begin().

## Receiving commands from the monitor

<img src="Report's images\Session04\code_buffer_serial_connection.png" width="600">

Then, I implemented a function that enables sending commands to the robots from the monitor using serial communication. I created a list of characters which will contain the command.

Whenever a character becomes available in the buffer, we read it and add it to the first empty position in the list (thanks to the increasing index position_character). When this character is a line break, we call a function that will parse the command.



# Parsing the command to get input parameters

<img src="Report's images\Session04\code_readCommand.png" width="600">

To parse the command, we compare the first characters with the expected results. Thus, if the first characters are "d=", we expect that the next characters, when converted to an integer, will represent the distance command. Similarly, if the first characters are "r=", it indicates that the subsequent characters will represent an angle value.

<img src="Report's images\Session04\code_correctDistance.png" width="600">


Finally, we have two error correction functions—one for correcting distance and another for correcting rotation. These functions will be called each time we acquire new measurements from the encoders. Both functions are very similar and based on the same principles. I will explain the correct_distance function.

Firstly, we apply the PID coefficients to the command, which has been calculated based on the difference between our previous command and the real position of the robot (refer to Session03 report). To achieve this, we increment the integral error with our new command. Subsequently, the command is simply equal to our PID correction at the maximum defined acceleration. We then check if this command is less than our maximum speed and if our acceleration is less than the maximum acceleration. If not, we limit it to our maximum values. The commands for the left motor and right motor will be the same because we correct the trajectory's orientation using the correct_orientation function.

Finally, we send our commands to the two motors using the motor commands function and we save our actual commands as previous commands.



## **Next session tasks:**

Before the next session, we should have completed the assembly of the new rolling base. This will enable me to implement my algorithm on the robot and calculate PID coefficients.
