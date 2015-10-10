# Lab 5: Mechanism Synthesis

**Released Thu. Oct. 8. Due Thu. Oct. 15, 15:30**

In this lab you will learn how to:
- Use dimensional synthesis to design four-bar linkages from work-space tasks
- Elaborate and rationalize these mechanisms to close the loop from task specification to manufacturable machine
- Use polymer bushings to improve the quality of revolute joints

## Exercises

To synthesize four-bar mechanisms, we have developed code which: given the lengths of a 2R serial chain, and a set of five points for the mechanism to pass through, generates a set of three fourbar mechanisms which accomplish the task. We are using the polynomial synthesis approach described in the "Path Generation of a Four-bar as a constrained 2R" document, written by Mark.

Our hope is that specifying a 2R chain, and the points it should move through allows for an intuitive definition of a task a four-bar mechanism should accomplish. Do give us some feedback on this point if you have an opinion.

Play around with the `code Austin will write` to familiarize yourself with the inputs and output of the synthesis function. Change the parameters of the 2R chain, change the points that the mechanism passes through.
### Walker
1. Lets create a walking mechanism. Define a 2R chain and a set of points that generates a continuous loop with a flat segment (like [this](https://en.wikipedia.org/wiki/Klann_linkage) for example). The mechanism you synthesize should be free of branch and circuit defects. In other words, the crank can turn continuously in one direction, and the output repeatedly traces the same curve.
2. Add your walking mechanism to a body (a simple cube would do) to create a little robot that can scoot around. Add two if you want to make something like [this](https://www.youtube.com/watch?v=RVgz6rnATM0). For actuation, you can use any motor you have on hand, or use a rubber band motor for a wind-up version.

## Design Challenge: Gripper

Let's make a gripper. For this task, we want to open and close the gripper with a single actuator. The grasping pads should move in and out in a straight line, and be able to fully close as shown in this image:

![Figure 2](https://github.com/CS194-028/starter/blob/master/lab_5/assets/gripper.jpg)

For your synthesis, try to optimize how wide the gripper can open so you can be able to pick up larger objects. To constrain the two sides such that they move together, you can use a [bell crank](http://www.robives.com/blog/bellcrankmech), a [crossed belt drive](http://www.expertsmind.com/questions/cross-belt-drive-30118956.aspx) made with a rubber band (make sure it is sufficiently stretched so avoid slippage, [crown your pulleys](https://www.youtube.com/watch?v=6sM0Qjumyro) if you want to get fancy), or a use gear pair (although I hardly encourage such behavior).

For the gripper, you can use the clevis joint primitive to create more accurate joints, as shown in this figure:


![Figure 1](https://github.com/CS194-028/starter/blob/master/lab_5/assets/clevis.jpg)
The clevis joint supports the rotational shaft from both sides, increasing the off-axis stiffness of the joint. It uses IGUS plain bearings (Part JFM-0304) to establish accurate, low friction surfaces. You can collect the shaft material, and IGUS parts from us; there is enough to go round.

To use: add the elements of the double support to one of the bodies the joint connects; add the center piece to the other. For assembly: press the IGUS bushings into the double support (this can be done by hand), then insert the center piece, then insert the shaft from the top. The shaft should be a friction fit in the center piece, so make sure you are componsating for any offsets resulting from 3d printer shrinkage, or laser cutter kerf. You can modify the primitive as you need it.

Make your gripper out of whatever manufacuring methods you would like. Completely 3D printed parts are fine, quality could  be improved (and machine time reduced) by using a combination of laser cut and 3D printed parts. Or, you can use folded structures and joints, as in lab 2.
