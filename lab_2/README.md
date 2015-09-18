# Lab 2: 2D Design and Fabrication

**Released Thu. Sep. 17. Due Thu. Sep. 24, 15:30**

2D fabrication is probably the most useful paragdigm you will use in this class. It can incorporate a variety of materials, the machine code is more simple and deterministic than 3D fabrication, and the parts are fast to manufacture.

Also running a laser cutter is way easier than the DI-WIRE.

In this lab you will learn how to:
* Design parts for fabrication with laser cutters (or waterjets, cnc routers)
* Add 2D joinery to elaborated designs
* Rationalize elaborated designs with planar slices
* Simulate your 2D fabricatable parts in assembled and flat configurations

## Exercises

### Slice forms
Let's look at how we would make a 3D solid out of 2D manufactured parts:

1. Complete `slice_mesh` to rationalize an imported stl with slice forms, including thru-holes you place for two 2mm diameter alignment pins. If using the supplied STL, rotate it 90 degrees about the x-axis for a better slicing result.

2. Simulate the slice formed solid in its assembled configuration by completing `slice_sim`.

When this is done, you can run the `slice_forms` function. This will produce `lab_2_slice_sheet_0.dxf` as well as `lab_2_slice_sim.scad`. The DXF file can be used on the Universal Laser Cutter. Before cutting, verify in illustrator that all of the line widths are 0.001. `lab_2_slice_sim.scad` will show you how the final rationalized solid will look.

### 2D joinery

We will inspect two of the many ways that 2D parts can be joined. The first
method is the [butt joint](https://en.wikipedia.org/wiki/Butt_joint). This is a
terrible joint. Mechanically speaking, it is weak and provides no registration
features which would align the parts to each other. Aesthetically, it is
somewhat...lacking. *However!* it's a simple place to start, and can be easily
modified into the [box joint](https://en.wikipedia.org/wiki/Finger_joint) by
cleverly spliting the intersecting geometry between the two bodies being joined. Make box joints if you can.

In the starter code are polygons which represent the exterior faces of an
uncomfortable looking chair. They are followed by a list of transformations
which transform linear\_extrude()s of those polygons into the 3D chair.

3. Write a function which elaborates a 3D object by extruding a list of polygons by a material thickness and transforming them into a final pose. The `quaternion_from_euler` or `euler_matrix` functions defined in `transformations.py` may be useful here.

4. Create a function which adds butt joints between planar solids. Rationalize the design by rotating the modified solids back to their original orientation and projecting then back onto the x-y plane. Offset these 2D projections to compensate for the kerf removed by the laser cutter (which can be measured from test cuts on your material, `offset_polygon` is written for you). Return a layout of parts ready for the laser cutter. Again label each part.

#### Folding

The other 2D joinery method we will use is origami-style folded joints. With folded joints, you can combined multiple different pieces into a single part. You can fold thick materials (including sheet metal) using kerf patterns like [these from industrial origami](http://www.industrialorigami.com/technology/gallery/cube/display.cfm?pic=cube_5&w=1103&h=1200). Assembly becomes much easier if you can fold your parts up. Instead of going to 3D CSG and projecting back into the plane to make these folded parts, we'll use 2D operations.

We will use two layers for folded parts, one for geometry which will be cut
through (red), and one which will be cut partially through on a lower power to
allow folding (black).

5. Write the `join_with_fold` function which joins a block and a PolyLine and adds a fold line between them. This function should operate by translating the second polyline such that the position of two ordered points is matched. As shown in this figure:

![Figure 1](https://github.com/CS194-028/starter/blob/master/lab_2/assets/joinf_fold_flow.jpg)

6. Write a function that adds a tab between two given points in a Block, using your `join_with_fold` function. The example object should look like this:

![Figure 2](https://github.com/CS194-028/starter/blob/master/lab_2/assets/example_fold.png) 

7. Assemble the chair with folded joints by sequentially joining the planar pieces with your functions. If any line which should be cut get lost in the union operations, you should add them back manually.


## Design Challenge

Make a parametric box! Your design function should accept length, width, and height arguments, as well as material thickness. You can use butt joints, box joints, or folded joints (or all of them!) to make the box. Add some art to it to spruce it up a bit.

## Deliverables

As always, we encourage you to make the objects you design. There is more oppurtunity for future innovation when you can look at a physical object in your hands. For this lab, the Jacobs material store isn't up yet, so I would reccomend making your objects out of corrugated cardboard. It's easy to cut and readily available.

1. Rendering of slice-formed solid, and dxf of 2D layout
2. Solution code for the 2D joinery compiler, including elaborator and joinery functions
- dxf of chair parts modified for butt joints (or box joints, if you can)
- The physical chair
3. Solution code which joins two polylines with a fold
- Solution code that adds tabs
- Folded layout of chair
3. Design code for parametric box

## Using the Laser Cutter in Jacobs

The laser cutter we will use is located in 110D Jacobs. Everyone who has completed **both** the GWS **and** the invention lab training is eligible to use it. It is still not fully set up so, there are a few caveats:

- Compressed air for the building is not operable, so we will be using the laser without compressed air. Because of this, it is especially important that users confirm the dust collection is on, and that the mirrors are cleaned when necessary.
- The local switch for the dust collection is not operable, technical staff (Design Specialist) are turning on the dust collection manually on the roof at the beginning of each work day. If users notice the ventilation is not on, please notify a Design Specialist.
- If you have **ANY** questions about how to use it, ask a design specialist right away. Chris is one such who works there full time, so you can ask for him. The machine is vulnerable in its current state so it's best to be careful.

As always post to Piazza when you have any questions, and may the odds be ever in your favor.

