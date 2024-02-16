# Session report 3:

**Session objective:** : 
- Assemble the solar panels (game accessories) in preparation for manufacturing the playfield.
- Develop the closed-loop motor control algorithm.

## Assembly of the 9 solar panels

The playfield for the competition comprises 9 elements referred to as "solar panels." These panels are arranged along one border of the table and these elements feature 2 colors : blue and yellow. The objective for the robots is to rotate as many solar panels as possible, aligning their team color towards the inside of the table to score points. Each solar panel consists of two 3D-printed parts, a vinyl cut, a screw, a nylstop nut, and a washer.

To meet the competition regulations, I initially sliced the 3D parts, adhering to the specified parameters outlined in the rules. It is crucial for our models to conform to similar weights and dimensions, ensuring approval by the regulations. This uniformity allows us to replicate the competition conditions with precision.

<img src="Report's images\Session03\2solarpanels_slicing.png" width="400">

That's why I notably needed to modify the fill density to 40% and the fill pattern to 'Cubic'.

<img src="Report's images\Session03\slicing_parameters.png" width="500">

Then, to assemble the two 3D-printed parts, I used a CHC M6*50mm screw with a Nylstop Nut, and placed a washer between the two parts to reduce friction, following the instructions in the plan. This assembly allows rotation between the 2 pieces.

<img src="Report's images\Session03\assembly_solarpanels_screw&nuts.jpg" width="300">
<img src="Report's images\Session03\assembly_instructions.png" width="300">

The main difficulty was that the nut was spinning freely when I tightened the assembly. Therefore, I had to wedge a flathead screwdriver between the plastic part and the nut while screwing.

Then, I cut our vinyl using a cutter to detach the shapes of the solar panels and glued them onto the 3D-printed part, ensuring the correct orientation.

<img src="Report's images\Session03\vinyl_carpet_accessories.jpg" width="300">

Finally, here are all the nine solar panels ready to be affixed to the table.

<img src="Report's images\Session03\9solarpanels_assembled.jpg" width="300">

## Programming the closed loop control

Finally i prepared an algorithm to control motors with a closed loop regulation. The commented code will be available on the github but here are some important points to understand it better : 

We define encoder pins as "INPUT_PULLUP" so if there is no signal on this pin, we will receive the value 1 and when we will reveive something it will be 0.

<img src="Report's images\Session03\code_input_pullup.png" width="400">

Then we will use interruptions so each time there is a changement on one of the left or right encoder pin, we will call the associated interrupt functions.

<img src="Report's images\Session03\code_interruption.png" width="500">

Finally as each coding wheel has a little diameter difference, we will never have the same number of tick per cm on each during translations and rotations. That's why we introduced the following variables : 

<img src="Report's images\Session03\code_tick_perCm&perRad.png" width="500">

The rest of the code with the pid regulation and odometry is well commented and available on the github.


## **Next session tasks:** 
Test the closed-loop control algorithm on the robot and select appropriate coefficients to estimate robot's position and to have a correct trajectory
