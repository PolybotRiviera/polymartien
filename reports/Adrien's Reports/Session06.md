# Session report 5:

**Session objective:** Test the rolling base with assembled belts, complete the assembly of the coding wheels, and then test them.



## Belts not strong enough.

First of all, after Loic finished assembling the belt transmissions, we tested the rolling base on the ground. However, one belt broke directly due to the starting torque of our motors. As a result, we concluded that sewing the belts might be risky in such a competition. To address this weakness, we identified some belts with the desired length that are already a closed loop, eliminating the need for sewing.

## Assembling the wheels with encoders.

<img src="Report's images\Session06\Coding_wheel_without_encoder.jpg" width="250">

Then, after I finished placing the encoder magnets in their 3D printed support, each of us assembled one coding wheel.

The assembly is mounted on an aluminum shaft cut to the desired size. It includes the wheel, an arm, and the 3D printed part that holds the magnet, with washers between them, all assembled by force with no tolerances. The encoder is mounted on the arm using the initial spacers and screws.

Then I mounted the other part of the arm on pivot connection on the rolling base with a bolt and washers. 

<img src="Report's images\Session06\Coding_wheel_assembled_rollingbase.jpg" width="250">


## Testing the encoders

I used the code I wrote previously to test the encoders on the coding wheels. I achieved a really good result for the left wheel, but the encoder of the right wheel was continuously detecting a variation in the magnetic field. As a result, the distance traveled by this wheel was increasing even when the wheel wasn't spinning.

At first, I thought it was due to a poorly fixed encoder that might be moving due to vibrations. However, even after correctly securing it, the results only slightly improved. I came to the conclusion that it was due to the magnet we broke, which was now made from three distinct parts glued together.

After some research, I discovered that because the magnet is broken into three parts, the fragmentation could lead to irregularities in the field, causing issues with the encoder's readings.

I tried replacing this magnet with a neodymium magnet I found, but the results were peculiar. On one side of the magnet, the encoder detected no variation, while on the other side, the encoder detected a significant variation even when the wheel wasn't spinning. 
I had a correct result only when placing the side of it in front of the hall sensor, which isn't the result desired.

Since our original magnet had a hole in the center, I wondered if this had an influence on the magnetic field. To verify this, I attempted to drill a hole in the new magnet using a drill press. However, it was challenging to keep the drill perpendicular without creating a specialized support.

<img src="Report's images\Session06\original_magnet.jpg" width="88"> Original Magnet

-

<img src="Report's images\Session06\tested_magnet.jpg" width="100"> Tested Magnet


To grasp the issue, I delved into understanding the precise workings of an encoder. A magnetic encoder detects rotational position changes through variations in the magnetic field. As the motor shaft's attached permanent magnet rotates, the magnetic sensor registers shifts in the magnetic field. But there are different way of magnetizing a magnet : it can be with a radial direction or axial direction as explained below.

<img src="Report's images\Session06\magnetization_field_lines_fromAKM.png" width="450">

The Hall element is positioned in an area where the horizontal magnetic field is uniform. So in front of a planar face with our original magnet.

This is likely the reason why our encoder wasn't detecting variations on our new magnet.

Since I couldn't find any documentation detailing the characteristics of the encoder's magnet for purchasing a replacement, I opted to contact the seller to inquire about this information. Eventually, they agreed to sell a new one independently.


## **Next session tasks:**
Once we receive the new magnet and the appropriate belts, our rolling base should be complete. I will then proceed to test it, make any necessary improvements, and begin the process of determining the PID coefficients.

## Sources 
https://www.akm.com/eu/en/products/rotation-angle-sensor/tutorial/magnetic-encoder/
