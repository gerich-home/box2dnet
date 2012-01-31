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
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/joints/WeldJoint.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/contacts/ChainAndCircleContact.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/contacts/ChainAndPolygonContact.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/contacts/EdgeAndCircleContact.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/contacts/EdgeAndPolygonContact.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/World.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/Island.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/joints/PrismaticJoint.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/joints/FrictionJoint.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/joints/MouseJoint.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/joints/RevoluteJoint.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/joints/DistanceJoint.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/contacts/Contact.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/contacts/ContactSolver.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/contacts/PolygonAndCircleContact.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/contacts/PolygonContact.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/dynamics/joints/RevoluteJointDef.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/pooling/IWorldPool.java
/trunk/jbox2d-library/src/main/java/org/jbox2d/pooling/normal/DefaultWorldPool.java
/trunk/jbox2d-library/src/test/java/org/jbox2d/utests/MathTest.java

2) Build library
3) Fix all problems with different behaviour of Java and .Net code.
4) Remove as much //UPGRADE_* commentS as I can.
5) Refactor code
6) Try to implement some test framework

Feel free to contribute.