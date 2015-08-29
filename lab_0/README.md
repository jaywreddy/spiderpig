# Lab 0: Software Infrastructure and Computational Geometry

**Released Thu. Sep. 3. Due Thu. Sep. 10, 15:30**

## Overview

In this lab you will learn how to

* Open and run code in a class Virtual Machine
* Use Git to check out starter code and DigiFab repositories
* Use Git-flavored Markdown to respond to exercise questions
* Use Git to check in responses and solution code
* Output standard geometry file types
* View geometry files in LibreCAD and OpenSCAD
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
a Piazz message to instructors with your GitHub username to the instructor.
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

1. Run the Virtual Machine image, login, open a terminal, and cd into the 
`comp_fab` directory.

1. Make sure you have the most recent version of the code by pulling from the 
starter repository: `git pull starter master`. You should also pull from the
digifab directory in the home folder.

1. Add your class repository as a remote with `git remote add origin your_url`,
replacing "your\_url" with the full path to your class repository.

1. Edit this `README.md` file to answer the following:
*Question 1: What is your name and GitHub username? Answer using a Markdown
table.*

1. Commit this change with `git commit -am "Adding name and username to README"`.

1. Push to your repo with `git push origin master`.

## Running code, viewing files

Now you are ready to run the starter code. The easiest way to do this is run
`solution.py` from the terminal in the lab\_0 folder by typing:

```
./solution.py
```

This should generate several files, among them being `lab_0.json`,`lab_0.dxf`,
and `lab_0.scad`.


### OpenSCAD

[OpenSCAD](http://www.openscad.org/documentation.html)

### LibreCAD

[LibreCAD](http://librecad.org/cms/home.html)

### Exercises

1. Run the starter code.

1. Open `lab_0.dxf` in LibreCAD. Use the Print to File option 

1. Open `lab_0.scad` in OpenSCAD.
*Question 2: What do you notice about the red shape compared to the DXF?*

1. Open `lab_0.json` in a text editor.
*Question 3: What*

## iPython, NumPy, SolidPython

You can also run the starter code from an iPython interpreter

[iPython](http://www.pythonforbeginners.com/basics/ipython-a-short-introduction)
tab completion, object?, running shell commands, mention notebooks

[SolidPython](https://github.com/SolidCode/SolidPython)
Python hooks to pretty much all OpenSCAD functions. Check out the utils module
for more useful functions.

[NumPy](http://cs231n.github.io/python-numpy-tutorial/)
nd indexing, hstack, vstack, dot, transpose, broadcast functions

### Exercises

1. Create geometry function like star.

1. Create SCAD geometry, maybe with text?

1. Create python function that will tile an input PolyLine

## Deliverables

1. Checked in `solution.py` that produces `lab_0.dxf` with new geometry functions. 
1. Checked in `README.md` with responses to lab questions.
1. Bring a printout of your `lab_0.dxf` to class.

