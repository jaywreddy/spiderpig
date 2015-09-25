# Lab 3: 3D Design and Fabrication

**Released Thu. Sep. 24. Due Thu. Oct. 1, 15:30**

## Exercises

Let's make a space frame structure mounted on a laser cut mounting plate. We'll start off with joinery methods for 1D and 2D rationalized structures using 3D printed materials. These joinery operations will be done with re-use of modular parametric solid objects.

[Space frames](https://en.wikipedia.org/wiki/Space_frame) are light-weight and rigid structures that can be found in most performance critical applications. Like Ducati motorcycles, and buildings that don't fall down. They consist of 1D truss elements joined together with brackets, or welded. For our purposes we will use 3D printed brackets, and whatever you like for your 1D elements. I would recommend bamboo kitchen skewers.

We have made a generator that produces a rough draft of the structure from a random seed. Your job in this lab will be to make this structure fully constructable using computational techniques. By being robust to a random seed, we can be fairly sure that our space-frame compiler will work for most designed objects.

The following directions are set up for using bamboo kitchen skewers (available at Safeway). If you want to use some other material for your struts, like laser-cut plastic or a stick you found on the ground, feel free to modify the lab to fit your purposes. Likewise, the connector geometry can be modified if you so choose.

## Code structure

There are four main classes in the starter code that you need to pay attention to. 

### Sculpture
The `Sculpture` class extends `Layout`. It generates a delaunay triangulation from of a supplied points cloud (if one is not supplied, it makes its own). From the edges of this triangulation, it generates `Stick` objects which will be our 1D trusses. It will also generate the `Connectors` between the `Sticks`, and a `Plate` for mounting the sculpture.

1. `Stick` generates a cylinder given a length and pose. Extend the `Stick` class to accept a diameter argument.

1. Modify the `Sculpture` class to generate `Sticks` with 3.0mm subtracted from each end, to avoid stick collisions at the node points.

### Connector
The `Connector` class extends `Body`. In its current form, it just places a 6mm diameter sphere at the node point. We will modify `Connector` so that it adds a round blind mortise joint for each `Stick` that the connector attaches.

1. In the Connector class, create a labelled blind mortise joint primitive according to the below figure:

![Figure 1](https://github.com/CS194-028/starter/blob/master/lab_3/assets/blind_mortise.JPG) 

2. For each `Stick` in the connector, add a blind mortise joint aligned to its axis.
3. Join all of the mortise joints with the original sphere. Ensure that all of the mortises for the `Sticks` stay empty. For help with this, see 
`solid_hole_example()` in the starter code.

There are three connectors that join the space frame to a base plate. For this joinery, we will add a 3D printed tenon to these connectors and cut a through mortise in the base plate.

1. Create a labelled, flanged tenon primitive as shown in the below figure:

![Figure 2](https://github.com/CS194-028/starter/blob/master/lab_3/assets/tenon.JPG) 

2. If the `Connector` attaches to the plate, then add a tenon to the connector. Again ensure that the area around the tenon is free of 3D printed material, so that it can be inserted into the laser cut plate.

### Plate
`Plate` creates a 2D elaborated object for mounting the rest of the sculpture. Modify the `Plate` class so that the tenons from the mating connectors get subtracted. Save a projection of the plate to a dxf.


Simulate the assembled structure with all of the final joints, and 1D trusses. To verify, check for intersection between the solids.

## Design challenge

Get creative with the point clouds you send to sculpture. Modify the way that edges are calculated from the point cloud (may I suggest a Voronoi partition?)

