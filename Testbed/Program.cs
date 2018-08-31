using System;
using System.Diagnostics;
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
            groundBodyDef.Position.Set(0, -10);
            Body groundBody = world.CreateBody(groundBodyDef);
            PolygonShape groundBox = new PolygonShape();
            groundBox.SetAsBox(50, 10);
            groundBody.CreateFixture(groundBox, 0);

            // Dynamic Body
            BodyDef bodyDef = new BodyDef();
            bodyDef.Type = BodyType.Dynamic;
            bodyDef.Position.Set(0, 4);
            Body body = world.CreateBody(bodyDef);
            PolygonShape dynamicBox = new PolygonShape();
            dynamicBox.SetAsBox(1, 1);
            FixtureDef fixtureDef = new FixtureDef();
            fixtureDef.Shape = dynamicBox;
            fixtureDef.Density = 1;
            fixtureDef.Friction = 0.3f;
            body.CreateFixture(fixtureDef);

            // Setup world
            float timeStep = 1.0f / 60.0f;
            int velocityIterations = 6;
            int positionIterations = 2;

            // Run loop
            var sw = new Stopwatch();

            sw.Start();
            const int iterations = 6000000;
            for (int j = 0; j < iterations; j++)
            {
                world.Step(timeStep, velocityIterations, positionIterations);
            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.TotalSeconds + " sec, " + );

            Vec2 position = body.Position;
            float angle = body.Angle;

            Console.WriteLine("{0:0.00} {1:0.00} {2:0.00}", position.X, position.Y, angle);
        }
    }
}
