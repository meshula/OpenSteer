![OpenSteer](doc/images/beta_250a.gif)

# OpenSteer: Steering Behaviors for Autonomous Characters 

### Feb. 2019

- Ported to glfw and Cmake

----------------

OpenSteer is a C++ library for constructing steering behaviors for autonomous characters in games and animation. 
In addition to the library, OpenSteer provides an OpenGL-based application called OpenSteerDemo which 
demonstrates a variety of steering behaviors. The user can quickly prototype, visualize, 
annotate and debug new steering behaviors by writing a plug-in for OpenSteerDemo.

![Demo](doc/images/typical_SteerTest.png)

OpenSteer provides a toolkit of steering behaviors, defined in terms of an abstract mobile agent called 
a "vehicle." Sample code is provided, including a simple vehicle implementation and examples of combining 
simple steering behaviors to produce more complex behavior. OpenSteer's classes have been designed to flexibly 
integrate with existing game engines by either layering or inheritance. 

OpenSteerDemo's plug-in framework allows a game AI programmer to quickly prototype behaviors during game 
design, and to develop behaviors before the main game engine is finished. OpenSteerDemo allows the user 
to interactively adjust aspects of the simulation. The user can: start, stop and single step time, select 
the vehicle/ character/ agent of interest, adjust the camera's view and its tracking behavior.

OpenSteer is distributed as open source software in accordance with the MIT 
License http://www.opensource.org/licenses/mit-license.php. OpenSteer was originally developed with the
generous  support of Sony Computer Entertainment America http://www.scea.com/. OpenSteer is supported
on Linux, Mac OS X and Windows.

The original discussion forum is on sourceforge. Discussion forum http://sourceforge.net/forum/forum.php?forum_id=264792
If you have issues or modifications please post them here on github in Issues, or as a Pull Request.

### Credits

OpenSteer was initially developed by Craig Reynolds beginning in 2002 at the Research and Development 
group of Sony Computer Entertainment America. The OpenSteer authors wish to acknowledge the support 
of SCEA, SCEI and in particular these executives who had the foresight to release this code as 
open source for all to use: Shin'ichi Okamoto, Masa Chatani and Dominic Mallinson.

After OpenSteer's source code was first released on May 1, 2003 a group of dedicated volunteers 
quickly formed and began to extend the system. Ports to Windows and Mac OS X were contributed within 
24 hours! Today we continue to discuss, test and improve OpenSteer. To contribute, or to just listen in, 
please visit the Open Discussion forum.

Here is a partial list of those who have contributed to OpenSteer, sorted roughly by the amount 
and "freshness" of their contributions: Bjoern Knafla (bknafla), Nick Porcino (meshula), 
Ben Forsyth (bforsyth), Dominik (inikofdoom), Paul (spanneradmin), Kris Hauser (kkhauser), 
Leaf Garland (leaf), Stefan Moises (beffy), Bruce Mitchener (brucem), Maciej Sinilo (yarpen), 
Michael Holm (gizwiz), Richard Cannock (perambulator).  Let us know of any omissions from this list.

Migrated from sourceforge http://opensteer.sourceforge.net


