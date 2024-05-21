# Session report 8: February 07, 2024

**Session objective:** Assemble the playground table, calibrate the robot's odometry, fix bugs in the algorithm, and test the robot on the ground.

# Before the session :

## Completing the assembly of our replica of the competition's playground.

Now that our five wooden boards and borders have been assembled, we were able to install planters and solar panels on the outside of the borders. To accomplish this, we, along with another team member, placed marks where the installations were needed as specified in the competition rules. For the planters, we utilized an angled drill guide, enabling us to securely fasten screws at an angle between the two wood planks.

<img src="Report's images\Session08\fixing_planters.jpg" width="250">

After completing most of the assembly, we were not yet prepared to apply the self-adhesive carpet because the planks were not perfectly aligned, which would result in significant bumps after installation. To address this issue, we added more screws to the smaller planks between the larger ones beneath the table. This adjustment was necessary as our previous screws were not positioned closely enough to the separations to prevent bending.

<img src="Report's images\Session08\aligning_wooden_planks.jpg" width="400">


## Applying the self-adhesive playmat.
Before applying the carpet, we cleaned the table with a cloth and soap, followed by 90° alcohol, to eliminate all dust particles. Indeed, even a tiny particle can create a visible bump on the carpet.

<img src="Report's images\Session08\cleaning_the_table.jpg" width="250">

Applying the three distinct self-adhesives took more than four hours, requiring three people to ensure proper installation. One person used a spatula to adhere the carpet, another stretched it on the opposite side, and the last one removed the sticker from the bottom. Achieving the desired result involved a slow and meticulous process, restarting each time a dust particle or large bubble appeared beneath the carpet. Unfortunately, the self-adhesive carpet did not have the correct specified dimensions due to a manufacturer's imprecision, resulting in incomplete coverage of the table.

I would like to thanks Frederic Juan from our school's Fablab for his help.

Here is the final result : 

<img src="Report's images\Session08\playground_table_finished.jpg" width="550">


## Final issue with the encoder
During my tests, one encoder's values were still oscillating even when the wheel wasn't moving. To resolve this issue, I simply adjusted the position of the magnet, moving it closer to the encoder. Additionally, I inserted washers between the magnet and other components to minimize backlash, which was causing vibrations.

<img src="Report's images\Session08\magnet_too_far_from_encoder.jpg" width="250">


# During the session : 

## First ground test of our robot.
To test the robot, I temporarily connected all the electronics, including a 24V battery, Cytron drivers, and the Arduino with rilsan collars. However, due to the weight being positioned at the back of the robot, its wheels were slipping, causing it to roll improperly. To address this issue, I added counterweights at the front of the robot to ensure proper ground adherence. (I temporarily used some boxes of screws for this purpose).

<img src="Report's images\Session08\robot_with_counterweights.jpg" width="500">

## Performing robot calibration.
Subsequently, I conducted a test of my algorithm. Initially, I calibrated the robot by measuring the number of ticks per centimeter and per radian. To achieve this, I placed the robot on the ground and manually moved it alongside a one-meter ruler in a straight line. After covering one meter, I read the number of ticks received by the Arduino and divided it by 100 to obtain the number of ticks per centimeter.

<img src="Report's images\Session08\calibrating_distances.jpg" width="250">

For rotation calibration, I rotated the robot 360 degrees, and then divided the obtained value by 2*Pi to determine the number of ticks per radian. 

This calibration enables me to calculate the distance traveled by the robot based on the number of ticks and correct our position until the desired one is reached. However, this method is not very precise, so we plan to soon manufacture a removable support that will be screwed onto the robot. This support will have a hole at the center of kinetics, allowing us to attach a board with a nail underneath the robot. This will enable us to rotate the robot in a perfectly circular manner.

Here is the result in rotation : https://youtube.com/shorts/9qRIA3YaiO8?si=xRvWADhP5XFraYfz

Here is the result in translation : https://youtube.com/shorts/vkulLAi6vmk?si=7tIgm1-a3TSwJvxI

These results were obtained without utilizing PID correction, only employing a basic corrector.

## Incorporating PID coefficients and implementing techniques to prevent oscillations.

Subsequently, I introduced PID coefficients into our error calculation. However, determining these coefficients proved challenging, as the robot's weight is soon to be modified with the addition of new actuators, electronics, and power supplies. This modification will significantly alter our correction coefficients.

Additionally, I came across a technique to prevent excessive oscillation around the desired value, thanks to videos from the YouTube channel 'Robot en carton.' The integral coefficient is employed to catch up with the robot's position. However, to avoid exceeding our target too much, we can set this coefficient to zero if the remaining angle to achieve is too large. As suggested in the videos, a remaining angle greater than PI/20 is considered suitable to disregard the integral coefficient.

## **Next session tasks:**
In the next session, I will work on the four cameras we just received, connecting them to the Nvidia Jetson, testing them, and utilizing them collectively to identify objects in all directions. Additionally, I will address absolute odometry; currently, our odometry is relative.
