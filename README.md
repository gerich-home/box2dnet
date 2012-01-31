Box2D.Net
=============

Hello everybody. This is plain port of JBox2D to .Net

This is converted using JLCA Java to C# code converter which you can find in Visual Studio 2005.
Now we need to fix all converting issues (there are not so much of them) and run it.

Main goal for me is to get a .Net version of Box2D which doesn't depend on XNA or other frameworks.
It will be better to call this project Plain Box2D.Net :)

Conversion of code was performed for revision 557 from http://code.google.com/p/jbox2d/source/browse/trunk/jbox2d-library/src/

I've already managed to compile library. Currently it's again unstable, because I perform syncing with actual code in svn

Current tasks:
1) Sync with 578.
Remained files:
dynamics/joints/WeldJoint.java
dynamics/contacts/ChainAndCircleContact.java
dynamics/contacts/ChainAndPolygonContact.java
dynamics/contacts/EdgeAndCircleContact.java
dynamics/contacts/EdgeAndPolygonContact.java
dynamics/World.java
dynamics/Island.java
dynamics/joints/PrismaticJoint.java
dynamics/joints/FrictionJoint.java
dynamics/joints/MouseJoint.java
dynamics/joints/RevoluteJoint.java
dynamics/joints/DistanceJoint.java
dynamics/contacts/Contact.java
dynamics/contacts/ContactSolver.java
dynamics/contacts/PolygonAndCircleContact.java
dynamics/contacts/PolygonContact.java
dynamics/joints/RevoluteJointDef.java
pooling/IWorldPool.java
pooling/normal/DefaultWorldPool.java
test/java/org/jbox2d/utests/MathTest.java

2) Build library
3) Fix all problems with different behaviour of Java and .Net code.
4) Remove as much //UPGRADE_* commentS as I can.
5) Refactor code
6) Try to implement some test framework

Feel free to contribute.