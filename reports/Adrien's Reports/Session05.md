# Session report 5:

**Session objective:** Assemble the pivot joints of the coding wheels and place the encoders in front of their axis.

To ensure continuous contact between our coding wheels and the ground, we opted for a pivot joint between the wheel and the rolling base, equipped with holes to attach an encoder using spacers. During this session, I began assembling the joint designed by Loic in the previous sessions.

## Recover the spacers and magnets initially present at the end of the motor shaft.

Firstly, I dismantled the encoders to reposition them in front of the coding wheels, aligning with our new design. Initially, I retrieved the spacers from the encoders, which will be reused to position them in front of the coding wheel axis.

<img src="Report's images\Session05\encoder_spacers.jpg" width="300">

Next, I dismantled the magnets from the ends of the two motor shafts (as circled in the photo) because we will need them in front of the encoders' new positions.

<img src="Report's images\Session05\magnet_motorshaft.jpg" width="300">

This was very challenging because the magnets were likely assembled while hot. As I needed more strength to remove them, I attempted to use pliers as a lever, but it didn't work. Eventually, I resorted to using a vise to firmly hold the motor by its lower shaft, preventing any damage to the motor shaft, while using another pair of pliers to stop it from turning on itself. After a considerable amount of time and with some assistance, I finally succeeded in recovering the magnets, even though one broke in the process.

<img src="Report's images\Session05\recovering_magnet.jpg" width="300">

## Modeling and 3D printing a component for assembling magnets and metal rods together.

We needed to position the magnets we recovered at the ends of our coding wheel metal rods. Gluing them together didn't seem like the best solution due to the rotational speed of the coding wheels. Losing the magnets during the competition would be dramatic, as it would prevent us from correcting our trajectories and tracking the robot's localization. Therefore, I decided to create a small 3D-printed part that will be used as a flush-link connection.

<img src="Report's images\Session05\3djoint_magnetmetalrod_1.png" width="200"><img src="Report's images\Session05\3djoint_magnetmetalrod_2.png" width="202">

Each part is forcefully inserted into a hole in the plastic component with zero tolerance. To enhance the fixation, we will even apply glue to the plastic part.

<img src="Report's images\Session05\metalrod&magnet_assembled.jpg" width="400">

## Cutting an aluminum rod to use it as an axis for the coding wheels.

Finally, I cut the rod into two parts of 15mm each using a hacksaw, so their length will be perfectly adapted to fit with the coding wheel pivot joint thickness.

<img src="Report's images\Session05\cutting_metalrod.jpg" width="300">

I also smoothed the cut surfaces with a metal file.

## **Next session tasks:**

In the next session, I will complete the assembly of the coding wheel fixation. If our rolling base is ready to move, I will test our algorithm to begin determining our PID coefficients.
