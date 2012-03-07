using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.ComponentModel;
using System.Windows.Threading;
using Box2D.Common;
using Box2D.Dynamics;
using Box2D.Collision.Shapes;
using System.Collections.ObjectModel;

namespace WpfTestbed
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public class BodyAdapter : INotifyPropertyChanged
        {
            private Body body;

            public BodyAdapter(Body body)
            {
                this.body = body;
            }

            public void Update()
            {
                RaisePropertyChanged("BodyX");
                RaisePropertyChanged("BodyY");
                RaisePropertyChanged("BodyAngle");
            }

            private void RaisePropertyChanged(string name)
            {
                if (PropertyChanged != null)
                {
                    PropertyChanged(this, new PropertyChangedEventArgs(name));
                }
            }

            public event PropertyChangedEventHandler PropertyChanged;

            public double BodyX { get { return body.Position.X * 50; } }
            public double BodyY { get { return body.Position.Y * 50; } }
            public double BodyAngle { get { return body.Angle * 180 / Math.PI; } }
        }

        public MainWindow()
        {
            Bodies = new ObservableCollection<BodyAdapter>();

            InitializeComponent();

            InitializeWorld();

            timer = new DispatcherTimer(TimeSpan.FromSeconds(timeStep), DispatcherPriority.Normal, (s, e)=>
            {
                world.Step(timeStep, velocityIterations, positionIterations);
                foreach (var body in Bodies)
                {
                    body.Update();
                }
            }, Dispatcher.CurrentDispatcher);
            timer.Start();
        }

        private void InitializeWorld()
        {
            // Static Body
            Vec2 gravity = new Vec2(0, -10);
            bool doSleep = true;
            world = new World(gravity);
            world.SleepingAllowed = doSleep;
            BodyDef groundBodyDef = new BodyDef();
            groundBodyDef.Position.Set(0, -10);
            Body groundBody = world.CreateBody(groundBodyDef);
            PolygonShape groundBox = new PolygonShape();
            groundBox.SetAsBox(50, 10);
            groundBody.CreateFixture(groundBox, 0);

            {
                // Dynamic Body
                BodyDef bodyDef = new BodyDef();
                bodyDef.Type = BodyType.Dynamic;
                bodyDef.Position.Set(5, 4);
                bodyDef.Angle = (float)(2 * Math.PI / 3);
                Body body = world.CreateBody(bodyDef);
                PolygonShape dynamicBox = new PolygonShape();
                dynamicBox.SetAsBox(1, 1);
                FixtureDef fixtureDef = new FixtureDef();
                fixtureDef.Shape = dynamicBox;
                fixtureDef.Density = 1;
                fixtureDef.Friction = 0.3f;
                body.CreateFixture(fixtureDef);
                Bodies.Add(new BodyAdapter(body));
            }

            {
                // Dynamic Body
                BodyDef bodyDef = new BodyDef();
                bodyDef.Type = BodyType.Dynamic;
                bodyDef.Position.Set(5, 10);
                bodyDef.Angle = (float)(Math.PI / 3);
                Body body = world.CreateBody(bodyDef);
                PolygonShape dynamicBox = new PolygonShape();
                dynamicBox.SetAsBox(1, 1);
                FixtureDef fixtureDef = new FixtureDef();
                fixtureDef.Shape = dynamicBox;
                fixtureDef.Density = 1;
                fixtureDef.Friction = 0.3f;
                body.CreateFixture(fixtureDef);
                Bodies.Add(new BodyAdapter(body));
            }

            {
                // Dynamic Body
                BodyDef bodyDef = new BodyDef();
                bodyDef.Type = BodyType.Dynamic;
                bodyDef.Position.Set(4.5f, 7);
                bodyDef.AngularVelocity = (float)(2 * Math.PI);
                Body body = world.CreateBody(bodyDef);
                PolygonShape dynamicBox = new PolygonShape();
                dynamicBox.SetAsBox(1, 1);
                FixtureDef fixtureDef = new FixtureDef();
                fixtureDef.Shape = dynamicBox;
                fixtureDef.Density = 1;
                fixtureDef.Friction = 0.3f;
                body.CreateFixture(fixtureDef);
                Bodies.Add(new BodyAdapter(body));
            }
        }

        ObservableCollection<BodyAdapter> bodies;
        public ObservableCollection<BodyAdapter> Bodies { get { return bodies; } private set { bodies = value; } }

        DispatcherTimer timer;

        World world;

        float timeStep = 1.0f / 60.0f;
        int velocityIterations = 6;
        int positionIterations = 2;
    }
}
