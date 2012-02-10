using System;
using Box2D.Collision.Shapes;
using Box2D.Common;
using Box2D.Dynamics;

namespace Testbed
{
    class Program
    {
        static void Main(string[] args)
        {
            // Static Body
            Vec2 gravity = new Vec2(0, -10);
            bool doSleep = true;
            World world = new World(gravity);
            world.SleepingAllowed = doSleep;
            BodyDef groundBodyDef = new BodyDef();
            groundBodyDef.position.set_Renamed(0, -10);
            Body groundBody = world.createBody(groundBodyDef);
            PolygonShape groundBox = new PolygonShape();
            groundBox.setAsBox(50, 10);
            groundBody.createFixture(groundBox, 0);

            // Dynamic Body
            BodyDef bodyDef = new BodyDef();
            bodyDef.type = BodyType.DYNAMIC;
            bodyDef.position.set_Renamed(0, 4);
            Body body = world.createBody(bodyDef);
            PolygonShape dynamicBox = new PolygonShape();
            dynamicBox.setAsBox(1, 1);
            FixtureDef fixtureDef = new FixtureDef();
            fixtureDef.shape = dynamicBox;
            fixtureDef.density = 1;
            fixtureDef.friction = 0.3f;
            body.createFixture(fixtureDef);

            // Setup world
            float timeStep = 1.0f / 60.0f;
            int velocityIterations = 6;
            int positionIterations = 2;

            // Run loop
            for (int i = 0; i < 60; ++i)
            {
                world.step(timeStep, velocityIterations, positionIterations);
                Vec2 position = body.Position;
                float angle = body.Angle;
                Console.WriteLine("{0:0.00} {1:0.00} {2:0.00}", position.x, position.y, angle);
            }
        }
    }
}
