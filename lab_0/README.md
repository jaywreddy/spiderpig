# Lab 0: Software Infrastructure and Computational Geometry
Question 1:

Name|Username
----|----
jay|jaywreddy

Question 2:
The bottom shape appears to be a triangular prism in OpenSCAD. However, in
librecad, the triangle is not actually closed and there appears to be a line
coming off of the bottom of the shape. I presume that OpenSCAD closed the shape
by joining the first and last points. Additionally, it appears that OpenSCAD
ignored or did not render the line on the bottom of the shape, which makes sense
since its a 0 width geometric object.

Question 3: When you type open_pl. and hit tab, how many member functions and
attributes are listed?
37

Question 4: What are the arguments and return type listed when you enter
 open_pl.hull??
 The only argument is a (float) margin.
 The return type is a PolyLine

Question 5: What is the output of open_pl.points[2:4,0], and what does this
 notation mean in terms of selecting a subset of the point coordinates?
 Output: array([100, 200])
 The notation specifies the indices that we would like to access in the
 n-dimensional array. So here we are accessing the 0th elements of the 2nd and
 3rd arrays.

Question 6: What OpenSCAD function correlates to the minus sign used to make
 squarcle_gen?
Difference

Question 7: How many polylines were in the PolyLine created with squarcle_gen
 before simplified() was called on it?
146

Question 8: Can an integer be used to scale a PolyLine object with the
 multiplication notation?
Yes

 Question 9: Which magic function and symbol has the equivalent behavior of the
  unioned function defined for the PolyLine class?
  __add__, +
