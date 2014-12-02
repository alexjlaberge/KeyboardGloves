##KeyboardGloves
Code for interpreting uart messages as keyboard input from our Keyboard Gloves for EECS 373 at UofM.
Also implements eyeLike for controlling mouse movements on Ubuntu.

##Status
src/mouseTest.cpp contains the code for controlling mouse position (and clicks if necessary) based on an x,y value.

src/serialTest.cpp contains the code for reading and sending data over FTDI.

src/main.cpp contains the code for getting the eye x,y values for use in the mouse program.

##Building

Xlib is required for building the mouse controller.
CMake is required to build KeyboardGloves.
