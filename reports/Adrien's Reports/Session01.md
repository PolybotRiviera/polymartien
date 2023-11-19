#Session Report 1:

**Session Objective:** Mount the received DC motors on the prototype, test them, and determine parameters useful for the robot's control.

During this session, we assembled the motors and wheels on the base so that I could initiate the control and odometry of the robot.

<img src="Report's images\Session01\rolling_base.jpg" width="200">

##Changes to the Rolling Base:
As mentioned in the project bibliography, the new version of our U-shaped robot has a large empty space in the center to collect plants in large quantities. This efficiency gain adds a significant constraint: we are now obliged to place the motors at the rear of the robot, as shown in the model created by Loïc below.
<img src="Report's images\Session01\URobot_3DModelisation.png" width="300">

However, for the robot to move easily and quickly avoid the opponent robot, it is preferable for the robot to be able to rotate on itself, and therefore have its wheels placed in the middle. Thus, we decided to shift the wheels by transmitting the rotation of the motors placed at the back of the robot to the wheels in the middle using belts.

My research also led me to realize that it would be risky to place encoders directly on the motor axes for robot odometry (i.e., to know its position based on the number of wheel rotations). In the event of robot slippage, the encoder may not be incremented even though the robot has moved, causing a drift in our estimation compared to the actual position. That's why many teams opt for encoder wheels. This involves placing encoders on free wheels parallel to the drive wheels. These free wheels are fixed via a pivot link so that the wheel can always remain in contact with the ground. Finally, the narrower the encoder wheels, the higher the odometry precision will be.
<img src="Report's images\Session01\Codingwheels_drawing_fromCUBOT.png" width="300">
<img src="Report's images\Session01\Codingwheels_liaison_drawing_fromCUBOT.png" width="300">
(Images Source : Cubot Team)


##Assembly and testing of motors and wheels, and encountered problems:
We were able to test our motors by connecting them using Cytron 13A CC Driver Cards.

This revealed a first problem: at high speed, the wheels shake perpendicular to the wheel plane. The most probable hypotheses are that this is due to wheel warping. We ordered roller wheels for good grip, but no precision information is provided on manufacturing tolerance. This will be verified with a comparator before the next session. Another possibility is that the motor axis is not perfectly perpendicular to the wheel axis. Finally, play within the motor can also be the cause, but it is much less than the rotation play of the wheel.

<img src="Report's images\Session01\wheel_on_robot.jpg" width="300">

##Control: Calculation of the coefficient of adhesion between the wheels and the table.

During this second part of the session, I sought to calculate the coefficient of adhesion of our wheels on the game table to use it in our control parameters. This will be essential (in addition to the center of gravity of the robot) to calculate very precisely the acceleration limit not to exceed under the risk of slipping. Indeed, to optimize acceleration and braking phases, the robot must be at all times at the slipping limit, considering a safety margin.
To measure this coefficient very precisely, the suspended bucket method can be used. This method simply requires a bucket, a pulley, a rope, and a scale.

<img src="Report's images\Session01\pulley_with_rope.jpg" width="200">
<img src="Report's images\Session01\hanging_seal_method.jpg" width="200">

It consists of attaching one end of the rope to the base of the robot placed on the table (as close as possible to the base), and the other end to a bucket suspended in the void by passing the rope through a pulley fixed to the end of the table. For this, we simply made a pulley using three 3D-printed pieces and a ball bearing. We then mechanically block the wheels, in our case simply using screwdrivers between the base and the outer surfaces of the wheels. The bucket is then filled with different objects serving as weights to apply a horizontal force on the robot.
To calculate this coefficient, proceed as follows:

<img src="Report's images\Session01\hanging_seal_method_illustration_fromRCVA.png" width="400">

- Measure the support force of the drive wheels on the ground by placing the robot wheels on a scale while keeping it horizontal with a wedge under the free wheel. Note this mass P1.
- Measure the mass of the objects in the bucket from which the robot is at the slipping limit, i.e., the force required to slide the robot with the wheels blocked. Note this mass P2.
- Finally, the coefficient is simply Ka = P2/P1.

Due to measurement inaccuracies (due to the choice of weights), you can calculate both the robot's adhesion coefficient unloaded and loaded and keep the average of the two. Indeed, the coefficient of adhesion is theoretically the same unloaded and loaded.

Having received the official vinyl game mat, we wanted to calculate the coefficient of adhesion between the wheels and the mat during this session. 
<img src="Report's images\Session01\playfield_vinyl_carpet.jpg" width="200">

However, our scale was limited to loads of 2 kg, so we have to postpone this calculation to later. However, this allowed us to observe the very strong adhesion of the roller wheels on the vinyl and to manufacture the suspended bucket system that will be used later.

##Sources:

    http://www.rcva.fr/wp-content/uploads/2016/12/devoir_de_vacances.pdf
    https://github.com/VRAC-team/la-maxi-liste-ressourceseurobot/blob/master/asservissement/Cubot-atelier_asservissement.pdf