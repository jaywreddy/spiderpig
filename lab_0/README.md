# Lab 0: Software Infrastructure and Computational Geometry

**Released Thu. Sep. 3. Due Thu. Sep. 10, 15:30**

## Overview

In this lab you will learn how to

* Open and run code in a class Virtual Machine
* Use Git to check out starter code and DigiFab repositories
* Use Git-flavored Markdown to respond to exercise questions
* Use Git to check in responses and solution code
* View geometry files in LibreCAD and OpenSCAD
* Print PDF files from LibreCAD
* Use iPython to generate and examine geometry objects
* Write Python code to generate and manipulate geometry objects

## VirtualBox

We have created a [VirtualBox](https://www.virtualbox.org/wiki/Downloads)
virtual machine image that is already set up with all of the software you
will need to run the labs. You can download the image file from the class
Piazza [Resources Page](https://piazza.com/berkeley/fall2015/cs194028/resources).
To run it, download the appropriate VirtualBox player from the website for
your system, and import the image file. The username and password is "maker"
and "CS194-028".

Since you will be generating files that need to be moved to protyping machines,
we suggest following these
[instructions](https://www.virtualbox.org/manual/ch04.html#sharedfolders)
for setting up a shared folder between your host and guest operating system.

For editing code, [Vim](www.vim.org), 
[Emacs](https://www.gnu.org/software/emacs/), 
[Sublime Text 3](www.sublimetext.com),
and [gedit](https://wiki.gnome.org/Apps/Gedit) are installed. You are welcome
to install any other software on the VM.

## Git

We will be using [GitHub](www.github.com) as our main method of distributing 
starter code, collecting solution code, and exercise responses.
If you are unfamiliar with Git, see this [tutorial](try.github.io)
to learn basic Git commands, and this more detailed 
[overview](https://git-scm.com/book/en/v2/Getting-Started-About-Version-Control)
for first-time users. [Markdown](https://help.github.com/articles/markdown-basics/)
is the text formatting language we will be using for exercies responses.

Before starting this lab, you should have created a GitHub account, and sent
a Piazza message to instructors with your GitHub username to the instructor.
We'll respond with a private repository that you will use for the remainder
of the class to submit lab work. All individual checkoffs will be based on 
what is checked into this repository by the due date and time.

After this first lab, you will be allowed to work in groups of two. Follow
up on the Piazza message with both student names and GitHub usernames with 
a note saying you will be working together. The instructor will grant push
and pull access for each student to both class repos. Each student will still
need to check in the solution to their own repository for full credit.

Generally each lab will have up to three submission components:

* Solution code that produces geometry files. Edit `solution.py` and check
in a working version of it.
* Response to questions in `README.md` file. These will be marked in italics as 
*Question N:*. Answer the questions by editing `README.md` and placing your
response directly follwing the question.
* A physical artifact produced from your geometry files that you bring to class.

Comitting early and often is encouraged. In general you should not check in 
any generated geometry or image files; we will run your checked-in code to 
generate these. By default these will be ignored by the `.gitignore` anyway.

### Exercises

1. Install VirtualBox.

2. Import the cs194-028.ova Virtual Machine in VirtualBox with File > Import 
Appliance ...

3. Run the VM by double-clicking it and open a terminal.

4. Make sure you have the most up to date DigiFab code by entering

```
cd ~/digifab
git pull origin master
```

You should do this every time you start working on solution code.

5. Enter the following commands to initialize your repository, replacing
the xx's with your team number.

```
git init ~/team_xx
cd ~/team_xx
touch README.md
git commit -am "Initializing"
git remote add origin https://github.com:CS194-028/team_xx.git
git remote add starter https://github.com:CS194-028/starter.git
git pull starter master
```

You should repeat the `git pull starter master` every time you work on code
as well.

5. Edit this `README.md` file to answer the following:
*Question 1: What is your name and GitHub username? Answer using a Markdown
table.*

6. Commit this change with `git commit -am "Adding name and username to README"`.

7. Push to your repo with `git push origin master`.

## Running code, viewing files

Now you are ready to run the starter code. The easiest way to do this is run
`solution.py` from the terminal in the lab\_0 folder by typing:

```
./solution.py
```

This should generate several files, among them being `all_shapes.dxf` and
`all_shapes.scad`.

### OpenSCAD

[OpenSCAD](http://www.openscad.org/documentation.html) is a Constructive
Solid Geometry (CSG) laguage with an integrated development and viewer
environment. Double clicking files with a `.scad` extension will open them
in in OpenSCAD.

### LibreCAD

[LibreCAD](http://librecad.org/cms/home.html) is a CAD program for viewing
and printing DXF files. This program also allows printing to PDF files with
scale-accurate dimensions. Double clicking files with a `.dxf` extension will
open them in LibreCAD.

### Exercises

1. Run the starter code.

2. Open `all_geometry.dxf` in LibreCAD. Enable Print Preview under the File
menu. Use the Print to File option to make a PDF. You might need to muck around
with the print scale and settings in Edit > Current Drawing Settings.

3. Open `all_geometry.scad` in OpenSCAD.
*Question 2: What do you notice about the bottom shape compared to the DXF?*

4. Try editing the geometry in the OpenSCAD window. A list of available
functions is [here](http://www.openscad.org/cheatsheet/index.html).

## Interactive Computational Fabrication

Since these exercises will focus on creating physical objects, it's important
to be able to quickly iterate on designs and see the the results. As an
intepreted language, Python naturally offers a paradigm for introspecting
and viewing live software objects.

### iPython

You can run fabrication code from an iPython interpreter. iPython is an
augmented command line interpreter for Python that has several useful
features such as:

* Tab completion of variable, attribute, function, filenames, and entry
history.
* Object and function documentation. Adding a question mark to the end of
any function will print the function signature and docstring for the function.
Two question marks will provide even more information, including the source
code of functions (hitting q will exit this view for long entries).
* Shell commands. Common Linux functions like `ls`,`pwd` and `cd` can be run
from the iPython interpreter.
* iPython [notebooks](http://ipython.org/notebook.html) offer a way of 
combining code exectution and document. We won't cover them here, but they're
worth checking out.

[Here](http://www.pythonforbeginners.com/basics/ipython-a-short-introduction)
is a tutorial for iPython for more information.

### NumPy

[NumPy](www.numpy.org) is a Python package for matrix operations and linear
algebra. The [DigiFab](https://github.com/CS194-028/digifab) library that 
represents geometry objects uses NumPy arrays for point and index variables.
Knowing how to manipulate NumPy arrays will help with the upcoming labs.
[Here](http://cs231n.github.io/python-numpy-tutorial/) is a tutorial
introduction. Multi-dimensional array indexing, 
[broadcasting](http://docs.scipy.org/doc/numpy/user/basics.broadcasting.html)
hstack, vstack, dot, and transpose are important functions to read up on.

### SolidPython

[SolidPython](https://github.com/SolidCode/SolidPython) is a Python package
that wraps all OpenSCAD functions in Python calls.

Python hooks to pretty much all OpenSCAD functions. Check out the utils module
for more useful functions.

### Exercises

1. Run an iPython interpreter by typing `ipython` in a shell.

2. Enter the first line of the `example` function in `solution.py` into the 
interpreter, and press enter. View the `open_pl` PolyLine by entering
`open_pl.show()` in the interpreter. You will have to close the viewing
window before entering more code.

3. Enter the rest of the `example` function lines one at a time (copy paste 
is your friend, which is ctrl-shift-c and ctrl-shift-v in the default Ubuntu 
shell). As you work through the code, answer the following questions about 
objects and functions.

*Question 3: When you type `open_pl.` and hit tab, how many member functions
and attributes are listed?*

*Question 4: What are the arguments and return type listed when you enter 
`open_pl.hull?`?*

*Question 5: What is the output of `open_pl.points[2:4,0]`, and what does 
this notation mean in terms of selecting a subset of the point coordinates?*

*Question 6: `print solid.scad_render(squarcle_gen)` will show the OpenSCAD 
string generated by `squarcle_gen`. What OpenSCAD function correlates to the 
minus sign used to make `squarcle_gen`?*

*Question 7: `len(squarcle_pl)` will return the number of unconnected polylines
in the PolyLine object. How many polylines were in the PolyLine created with 
`squarcle_gen` before `simplified` was called on it?*

*Question 8: Can an integer be used to scale a PolyLine object with the
multiplication notation?*

*Question 9: Python uses so-called "magic functions" to implement the behavior
of symbolic in-fix notation (i.e. `foo + bar` is equivalent to 
`foo.__add__(bar)`. You can see all of the magic functions defined for an 
object by typing entering `obj.__?`. Which magic function and symbol has the 
equivalent behavior of the `unioned` function defined for the PolyLine class?*

## Design Challenge

Complete the following functions in `solution.py`. When you meet the minimum
requirements for checkoff, the `solution` function will no longer print any
lines like `Something's wrong with ...`. You should still display the results
of each of your functions to check that they produce what you expect.

### Exercises

1. `combine_shapes`: Write a function that accepts a list of shapes created in
the `example` function and applies the same transforms using only OpenSCAD
functions. This means no using any functions defined in the Geometry or
PolyLine class (other than constructors).

2. `tile_shape`: Given a PolyLine shape and number of elements in x and y
directions, produce a rectangular grid of the shape. Make sure that there is
enough of a margin between the shapes that they can't intersect.

3. `make_art`: Get your creative juices flowing and create a design with the
functions you've learned so far. The only requirements are that it have at
least 10 individual polyline elements, and 100 points. You'll bring a printout
of this design to class, and have an opportunity to explain the functions and
techniques you used.

## Deliverables

* Checked in `solution.py` with completed functions that passes all assertion
tests in `solution`.

* Checked in `README.md` complete with responses to questions.

* Bring a printout of your computationally fabricated artwork to class.

