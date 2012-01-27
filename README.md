Box2D.Net
=============

Hello everybody. This is plain port of JBox2D to .Net

This is converted using JLCA Java to C# code converter which you can find in Visual Studio 2005.
Now we need to fix all converting issues (there are not so much of them) and run it.

Main goal for me is to get a .Net version of Box2D which doesn't depend on XNA or other frameworks.
It will be better to call this project Plain Box2D.Net :)

First I want to fix all compilation errors, then I will fix all problems with different behaviour of Java and .Net code.

Currently I am reworking every file to remove as much //UPGRADE_* comment as I can.

Conversion of code was performed for revision 557 from http://code.google.com/p/jbox2d/source/browse/trunk/jbox2d-library/src/
When I'll complete all fixes I'll start syncing changes from it with our code.

Feel free to contribute.