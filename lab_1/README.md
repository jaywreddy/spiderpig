# Lab 1: 1D Design and Fabrication

**Released Thu. Sep. 10. Due Thu. Sep. 17, 15:30**

In this lab you will learn how to:
* Design parts for fabrication with the DI-WIRE CNC wirebender 
* Add joinery to elaborated designs
* Verify that the design files are manufacturable given machine limits
* Simulate the output of the DI-WIRE given its input

## 1D Fabrication

This first lab introduces the computational design of structures made out of bent wire. Wire bending yields just about the best simplicity to capability ratio of all manufacturing processes. Low startup costs and dead-simple supply chains make it a very attractive option.

Our tool will be the DI-WIRE CNC wirebender, which is currently located in the Invention Lab. Explore some of the project on the [Pensa Labs site](http://www.pensalabs.com/bendables/) to see what people have used the machine for. Note that this machine can only do 2D bends, whereas production machines would be full 3D.

The [user manual](http://static1.squarespace.com/static/53ac5d6ee4b02ecfa224b242/t/54611765e4b043f3ac05a1bc/1415649125267/DIWire_User_Manual.pdf) covers the operation of the machine and driver software. The "Wireware" software, which runs the machine, makes toolpaths from imported dxfs which we will generate. It can do some interpretation of curved lines, but it doesn't do a great job. So our strategy will be to generate the exact points that will be bent by the machine.

## Joinery for 1D structures

Joinery for bent wire structures is accomplished by connecting adjacent pieces of wire. We can't solder because we don't have a permit to do hot work in Jacobs. So for simplicity we will use these [parallel clips from Pensa](http://www.pensalabs.com/accessories/parallel-clips).

The thickness of the wire has to be accounted for in bent wire designs so parts will have to be offset by a certain amount to fit properly. Figure 1 demonstrates how this would work for structures comprised of adjacent planar faces.

![Figure 1](https://github.com/CS194-028/starter/blob/master/lab_1/assets/wire_offset.jpg) 

The first figure shows the structure we want to make, three triangular faces coming together at a point. The second pane shows the fully rationalized design with offsets to allow for wire thickness. The third plane is a section view through a wire section showing how the rationalized pieces fit to the specified design.

*Question: what is the formula for this offset, based on the dihedral angle between the adjacent planes and the radius of the wire?*


## Exercises

1. Create design files for the triangular faces of a tetrahedron with appropriate offsets for joinery, and end termination [figure 1]
2. Write a verification function that checks that no two points of a polyline are within 12mm of each other, and no individual bend exceeds 135 degrees. This will tell you if your design file is manufacturable and save you machine time and material cost.
3. Create a function which simulates the output of the DI-WIRE given a 3D polyline and a 0.125" wire diameter
4. Let's make 3D bent structures like this 3D phone holder:

![Figure 2](https://github.com/CS194-028/starter/blob/master/lab_1/assets/bend_fig.jpg)

The black lines represent the folded wire that would be produced by the DI-WIRE, in other words the design file.

 from the following points (which are in inches):

[[3.0568,0.0,0.4915], [3.4,0.0,0.0], [2.8,0.0,0.0], [1.6405,0,1.6405], [0.0,0.0,0.0], [0.0,2.0,0.0], [1.6405,2.0,1.6405], [2.8,2.0,0.0], [3.4,2.0,0.0], [3.0568,2.0,0.4915]]  

Create a function which unfolds a 3D polyline so that the DI-WIRE can manufacture it. It should bend the open angle for you such that the final deformation is purely torsional as shown in figure 2. This strategy will allow you to make accurate 3D structures. Try to avoid unfolded configurations which intersect with themselves, as this may crash the wire into the head of the wire bender.

## Design Challenge

Make structural wire supports for a set of shelves described by a list of endpoints i.e. shelves = [[p0-p3], [p0-p3],...]. Not every set of input point has to work, but your code should throw an exception if the shelf supports cannot be fabricated.
Simulate your design and ensure that it passes the DFM check.

![Figure 3](https://github.com/CS194-028/starter/blob/master/lab_1/assets/shelves.jpg)

Starter points:

[[[0.0,0.0,50.0],[100.0,0.0,50.0],[100.0,50.0,50.0],[0.0,50.0,50.0]],
[[100.0,0.0,75.0],[150.0,0.0,75.0],[150.0,50.0,75.0],[100.0,50.0,75.0]],
[[0.0,0.0,100.0],[100.0,0.0,100.0],[100.0,50.0,100.0],[0.0,50.0,100.0]]]

## Deliverables

* Solution code that
  * Produces stl file of tetrahedron
  * Verifies manufacturability
* Wire-bent tetrahedron
* Shelves for chosen points, demonstration of error catching

## Extensions

* Extend your tetrahedron exercise to generalized polygonal wireframes.
* Add a space-filling curve to a polygon
* Instead of throwing an error, allow your verification code to split polylines into manufacturable pieces. Modify for joinery.
