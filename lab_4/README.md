# Lab 4: Mechanism Design and Fabrication

**Released Thu. Oct. 1. Due Thu. Oct. 8, 15:30**

In this lab you will learn how to:
- Use the digifab library for kinematics
- Represent mechanisms and their motion with digifab
- Computationally design mechanisms given a kinematic skeleton representation
  * By using pre-designed rotational joint primitives
  * By placing joints and shaping links to avoid self-intersecting geometry
 
## Open Kinematic Chains

For this part of the lab, you will make a torso with movable arms for the robot
figure you laser cut and assembled in Lab 2.

The starter code defines a Robot Mechanism that has a torso Body, and two
placeholder arms attached to the shoulder area of the torso at 45 degrees.
Running `r = Robot(); r.show()` should display this Mechanism in the default
configuration. Try running `r.show([0.0,-0.5,0.5])` to see the arms move
to a new angular state as shown below.

Default Configuration:
![Figure 1](https://github.com/CS194-028/solution/blob/master/lab_4/assets/robot_front.png)

Moved Configuration:
![Figure 2](https://github.com/CS194-028/solution/blob/master/lab_4/assets/robot_moved.png)

This torso has an automatically generated mortise slot for the lower part of 
the robot figure, which is implemented in `add_mortice`. You will use a similar
method to implement `add_snap_joint`, which will modify existing geometry with
joint definitions to use pre-defined connection geometry. `snap_shaft` returns
a snap shaft and hole geometry that can be used to 3D print rotational 
connections. The hole can either be 3D printed, or laser cut from appropriate
thickness material.

Automatically Generated Mortise:
![Figure 3](https://github.com/CS194-028/starter/blob/master/lab_4/assets/robot_back.png)

Snap Shaft:
![Figure 4](https://github.com/CS194-028/starter/blob/master/lab_4/assets/snap_shaft.png)

Snap Hole:
![Figure 5](https://github.com/CS194-028/starter/blob/master/lab_4/assets/snap_hole.png)

### Deliverables

1. Implement the Arm class to produce a Mechanism with two bodies (an upper arm
and a forearm) connected by a joint, that can connect to the torso at the base 
of the upper arm.
2. Add two Arms as children to the Robot class, and change it's connections so that
it connects these arms to the torso at the shoulder joints.
3. Implement `add_snap_joint`. Given two bodies connected by a rotational joint,
this function will return modified geometry to add the snap shaft and remove
material around the shaft to one part, and add a hole to the other part.
4. Use `add_snap_joint` to modify the torso and arm geometries to use the
included snap joint designs.
5. 3D print the torso and arm parts, and bring the assembled torso to class.

## Closed Kinematic Chains

Let's make a simple closed chain mechanism, [the pantograph](https://en.wikipedia.org/wiki/Pantograph). The pantograph is linkage which uses parallel motion to create scaled versions of a traced input. It was very popular with [engravers](http://www.engraversjournal.com/article.php/2207/index.html) for a long time. Here is a diagram of the pantograph:

![Figure 1](https://github.com/CS194-028/starter/blob/master/lab_4/assets/pantograph.jpg)

To esure proper operation, the pantograph has to be a *parallel mechanism*, meaning that links A and B are parallel, and C and D are parallel. For the motion of the output to be a pure scaling of the input, the input must be placed on a line drawn from the ground pivot to the output. The scale is determined by the ratio of: the distance from the output to the ground pivot, to the distance from the input to the ground pivot.

For laser-cur links, the snap shaft has been design to fit 4mm diameter holes cut in 1/8" thick material

### Deliverables

1. Edit the `Pantograph` class to calculate link lengths, given a scale factor argument.
2. Elaborate the pantograph mechanism with the provided snap joints, and laser cut link bodies

## Design Challenge

Pick out an interesting linkage, and make it in digifab. Here are a few sources to get you started:

[Youtube](https://www.youtube.com/user/thang010146/videos)

[KMODDL](http://kmoddl.library.cornell.edu/model.php?cat=S)

[Some artists](https://en.wikipedia.org/wiki/Kinetic_art#Selected_kinetic_sculptors)
