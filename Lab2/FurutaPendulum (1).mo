model FurutaPendulum "Furuta pendulum"
    
    parameter Real pendulumA_start = -Modelica.Constants.pi;
    
    model ControllerLQR
      Modelica.Blocks.Interfaces.RealInput phi, dphi, theta1, dtheta1, theta2, dtheta2;
      Modelica.Blocks.Interfaces.RealOutput u(start=0);
      Real x[6];
    equation
      x = {phi+3.14/2, dphi, theta1+3.14, dtheta1, theta2+3.14, dtheta2};
    end ControllerLQR;
    
    inner Modelica.Mechanics.MultiBody.World world(
      axisLength(displayUnit="mm"),
      axisDiameter(displayUnit="mm"),
      nominalLength(displayUnit="mm") = 0.1)
                                annotation (Placement(transformation(extent={{-114.66666666666667,-33.599999999999994},{-94.66666666666667,-13.599999999999994}},   rotation=0.0,origin = {0.0,0.0})));
    Modelica.Mechanics.MultiBody.Joints.Revolute rotor(
      a(fixed=false),
      w(fixed=true),
      cylinderLength(displayUnit="mm") = 0.015,
      cylinderColor={0,0,0},
      cylinderDiameter(displayUnit="mm") = 0.0605,
      useAxisFlange=true,
      n={0,1,0},
      phi(fixed=true, start=-1.5707963267949)) annotation (Placement(
          transformation(
          origin={-54.66666666666667,58.400000000000006},
          extent={{-10.0,-10.000000000000004},{10.0,10.000000000000004}},
          rotation=90.0)));
    Modelica.Mechanics.MultiBody.Joints.Revolute pendulumAxis(
      a(fixed=false),
      w(fixed=true),
      cylinderLength(displayUnit="mm") = 0.005,
      cylinderDiameter(displayUnit="mm") = 0.005,
      cylinderColor={200,200,200},
      useAxisFlange=true,
      n={-1,0,0},
      phi(
        fixed=true,
        start=3.14159265358978999667,
        displayUnit="rad")) annotation (Placement(transformation(extent={{-16.666666666666668,82.4},{3.333333333333332,102.4}}, rotation=0.0,origin = {0.0,0.0})));
    Modelica.Mechanics.MultiBody.Parts.BodyCylinder base(
      r(displayUnit="mm") = {0,0.1,0},
      r_shape(displayUnit="mm") = {0,0,0},
      diameter(displayUnit="mm") = 0.06,
      color={155,155,155},
      r_0(displayUnit="mm", fixed=true)) annotation (Placement(transformation(
          extent={{-10.0,-10.0},{10.0,10.0}},
          rotation=90.0,
          origin={-54.66666666666667,0.3999999999999986})));
    Modelica.Mechanics.MultiBody.Parts.BodyCylinder pendulumArm(
      r_shape(displayUnit="mm") = {0,0,0},
      diameter(displayUnit="mm") = 0.005,
      color={200,200,200},
      r(displayUnit="mm") = {0,0.075,0},
      density=3700) annotation (Placement(transformation(
          extent={{-10.0,-10.0},{10.0,10.0}},
          rotation=-90.0,
          origin={15.333333333333329,28.4})));
    Modelica.Mechanics.MultiBody.Parts.Fixed fixed(
      length=0,
      width=0,
      height=0,
      r(displayUnit="mm") = {0,-0.025,-0.1})
      annotation (Placement(transformation(extent={{-10.0,-10.0},{10.0,10.0}},
          rotation=90.0,
          origin={-54.66666666666667,-25.599999999999994})));
    Modelica.Mechanics.Rotational.Components.Damper pendulumDamper(d=0.05)
      annotation (Placement(transformation(
          extent={{-10.0,-10.0},{10.0,10.0}},
          rotation=180.0,
          origin={-6.666666666666668,112.4})));
    Modelica.Mechanics.Rotational.Components.Damper rotorDamper(d=0.05) annotation (
        Placement(transformation(
          extent={{-10.0,-10.0},{10.0,10.0}},
          rotation=180.0,
          origin={-78.66666666666667,68.4})));
    Modelica.Mechanics.MultiBody.Parts.BodyCylinder pendulumAttachment(
      r_shape(displayUnit="mm") = {0,0,0},
      diameter(displayUnit="mm") = 0.005,
      color={155,155,155},
      r(displayUnit="mm") = {0.043,0,0},
      density=3700) annotation (Placement(transformation(
          extent={{-48.0,80.0},{-28.0,100.0}},
          rotation=0.0,
          origin={0.0,0.0})));

    Modelica.Mechanics.Rotational.Sensors.AngleSensor pendulumA
      annotation (Placement(transformation(extent={{23.33333333333333,90.4},{37.33333333333333,104.4}},rotation = 0.0,origin = {0.0,0.0})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor pendulumW
      annotation (Placement(transformation(extent={{23.33333333333333,106.4},{37.33333333333333,120.4}},rotation = 0.0,origin = {0.0,0.0})));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor rotorA
      annotation (Placement(transformation(extent={{-74.66666666666667,14.399999999999999},{-60.66666666666667,28.4}},rotation = 0.0,origin = {0.0,0.0})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor rotorW
      annotation (Placement(transformation(extent={{-74.66666666666667,28.4},{-60.66666666666667,42.4}},rotation = 0.0,origin = {0.0,0.0})));
    Modelica.Blocks.Sources.Pulse pulse[3](
      startTime={1,1,1},
      width={1,1,1},
      period={10,10,10},
      amplitude={0.025,0,0})   annotation (Placement(transformation(
          extent={{-28.666666666666668,-11.600000000000001},{-16.666666666666668,0.3999999999999986}},
          rotation=0.0,
          origin={0.0,0.0})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce disturbance
      annotation (Placement(transformation(extent={{-4.666666666666671,-13.600000000000001},{11.333333333333329,2.3999999999999986}},rotation = 0.0,origin = {0.0,0.0})));
    .Modelica.Mechanics.MultiBody.Parts.BodyCylinder pendulumAttachement_2(r_0(start = {0.0,0,0}),r(start = {0.1,0,0}) = {0.043,0,0},r_shape = {0.0,0,0},diameter = 0.005,density = 3700,color = {155,155,155}) annotation(Placement(transformation(extent = {{-66.0,-118.0},{-46.0,-98.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Mechanics.MultiBody.Parts.BodyCylinder pendulumArm_2(diameter = 0.005,density = 3700,color = {200,200,200},r = {0,0.03,0}) annotation(Placement(transformation(extent = {{36.070363560218894,-116.31649819880333},{59.929636439781106,-99.68350180119667}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Mechanics.MultiBody.Joints.Revolute revolute(useAxisFlange = true,phi(start = 3.14159265358978999667),cylinderLength = 0.005,cylinderDiameter = 0.005,cylinderColor = {200,200,200},n = {-1,0,0}) annotation(Placement(transformation(extent = {{-6.0,-118.0},{14.0,-98.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Mechanics.Rotational.Components.Damper damper(d = 0.025) annotation(Placement(transformation(extent = {{13.478180595976475,-66.39734767910947},{-9.478180595976475,-41.60265232089053}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation(Placement(transformation(extent = {{66.77706570572273,-97.22293429427727},{81.22293429427727,-82.77706570572273}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(Placement(transformation(extent = {{67.99184076714495,-64.00815923285505},{80.00815923285505,-51.99184076714495}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(Placement(transformation(extent = {{99.37940400128075,-98.0},{80.62059599871925,-118.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Mechanics.MultiBody.Joints.Revolute rotor2(phi(fixed = true,start = -1.5707963267949),n = {0,-1,0},useAxisFlange = true,cylinderDiameter(displayUnit = "mm") = 0.0605,cylinderColor = {0,0,0},cylinderLength(displayUnit = "mm") = 0.015,w(fixed = true),a(fixed = false)) annotation(Placement(transformation(origin = {-248.8884788755796,-57.9567933655931},extent = {{-10,-10},{10,10}},rotation = 90)));
    .Modelica.Mechanics.Rotational.Components.Damper rotorDamper2(d = 0.05) annotation(Placement(transformation(extent = {{-10,-10},{10,10}},rotation = 180,origin = {-272.8884788755796,-47.9567933655931})));
    .Modelica.Mechanics.Rotational.Sensors.AngleSensor rotorA2 annotation(Placement(transformation(extent = {{-268.8884788755796,-101.95679336559311},{-254.8884788755796,-87.95679336559311}},rotation = 0,origin = {0,0})));
    .Modelica.Mechanics.Rotational.Sensors.SpeedSensor rotorW2 annotation(Placement(transformation(extent = {{-268.8884788755796,-87.95679336559311},{-254.8884788755796,-73.95679336559311}},rotation = 0,origin = {0,0})));
    equation
    connect(pendulumAxis.frame_b, pendulumArm.frame_a) annotation (Line(
        points={{3.3333333333333286,92.4},{15.333333333333329,92.4},{15.333333333333329,38.4}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(base.frame_b, rotor.frame_a) annotation (Line(
        points={{-54.66666666666667,10.399999999999999},{-54.66666666666667,48.4}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(base.frame_a, fixed.frame_b) annotation (Line(
        points={{-54.66666666666667,-9.600000000000001},{-54.66666666666667,-15.600000000000001}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pendulumDamper.flange_b, pendulumAxis.support) annotation (Line(
        points={{-16.66666666666667,112.4},{-24.66666666666667,112.4},{-24.66666666666667,102.4},{-12.666666666666671,102.4}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(pendulumDamper.flange_a, pendulumAxis.axis) annotation (Line(
        points={{3.3333333333333286,112.4},{11.333333333333329,112.4},{11.333333333333329,102.4},{-6.666666666666671,102.4}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotorDamper.flange_a, rotor.axis) annotation (Line(
        points={{-68.66666666666667,68.4},{-64.66666666666667,68.4},{-64.66666666666667,58.4}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotorDamper.flange_b, rotor.support) annotation (Line(
        points={{-88.66666666666667,68.4},{-94.66666666666667,68.4},{-94.66666666666667,52.4},{-64.66666666666667,52.4}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(pendulumAttachment.frame_b, pendulumAxis.frame_a) annotation (Line(
        points={{-28,90},{-28,92.4},{-16.66666666666667,92.4}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pendulumAttachment.frame_a, rotor.frame_b) annotation (Line(
        points={{-48,90},{-54.66666666666667,90},{-54.66666666666667,68.4}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pendulumW.flange, pendulumAxis.axis) annotation (Line(
        points={{23.33333333333333,113.4},{19.33333333333333,113.4},{19.33333333333333,102.4},{-6.666666666666671,102.4}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(pendulumA.flange, pendulumAxis.axis) annotation (Line(
        points={{23.33333333333333,97.4},{19.33333333333333,97.4},{19.33333333333333,102.4},{-6.666666666666671,102.4}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotorW.flange, rotor.axis) annotation (Line(
        points={{-74.66666666666667,35.4},{-78.66666666666667,35.4},{-78.66666666666667,58.4},{-64.66666666666667,58.4}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotorA.flange, rotor.axis) annotation (Line(
        points={{-74.66666666666667,21.4},{-78.66666666666667,21.4},{-78.66666666666667,58.4},{-64.66666666666667,58.4}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(disturbance.frame_b, pendulumArm.frame_b) annotation (Line(
        points={{11.333333333333329,-5.600000000000001},{15.333333333333329,-5.600000000000001},{15.333333333333329,18.4}},
        color={95,95,95},
        thickness=0.5,
        smooth=Smooth.None));
    connect(pendulumAttachement_2.frame_b,revolute.frame_a) annotation(Line(points = {{-46,-108},{-6,-108}},color = {95,95,95}));
    connect(revolute.frame_b,pendulumArm_2.frame_a) annotation(Line(points = {{14,-108},{36.070363560218894,-108}},color = {95,95,95}));
    connect(damper.flange_a,revolute.axis) annotation(Line(points = {{13.478180595976475,-54},{36,-54},{36,-90},{4,-90},{4,-98}},color = {0,0,0}));
    connect(force.frame_b,pendulumArm_2.frame_b) annotation(Line(points = {{80.62059599871925,-108},{59.929636439781106,-108}},color = {95,95,95}));
    connect(angleSensor.flange,revolute.axis) annotation(Line(points = {{66.77706570572273,-90},{4,-90},{4,-98}},color = {0,0,0}));
    connect(speedSensor.flange,revolute.axis) annotation(Line(points = {{67.99184076714495,-58},{67.99184076714495,-82},{48,-82},{48,-98},{4,-98}},color = {0,0,0}));
    connect(revolute.support,damper.flange_b) annotation(Line(points = {{-2,-98},{-2,-92},{-34,-92},{-34,-54},{-9.478180595976475,-54}},color = {0,0,0}));
    connect(force.force,pulse.y) annotation(Line(points = {{101.2552848015369,-108},{107.2552848015369,-108},{107.2552848015369,-22},{-12,-22},{-12,-5.600000000000001},{-16.06666666666667,-5.600000000000001}},color = {0,0,127}));
    connect(disturbance.force,pulse.y) annotation(Line(points = {{-6.266666666666673,-5.599999999999994},{-16.06666666666667,-5.600000000000001}},color = {0,0,127}));
    connect(rotorDamper2.flange_a,rotor2.axis) annotation(Line(points = {{-262.8884788755796,-47.9567933655931},{-258.8884788755796,-47.9567933655931},{-258.8884788755796,-57.95679336559311}},color = {0,0,0}));
    connect(rotorDamper2.flange_b,rotor2.support) annotation(Line(points = {{-282.8884788755796,-47.9567933655931},{-288.8884788755796,-47.9567933655931},{-288.8884788755796,-63.95679336559311},{-258.8884788755796,-63.95679336559311}},color = {0,0,0}));
    connect(rotorW2.flange,rotor2.axis) annotation(Line(points = {{-268.8884788755796,-80.95679336559311},{-272.8884788755796,-80.95679336559311},{-272.8884788755796,-57.95679336559311},{-258.8884788755796,-57.95679336559311}},color = {0,0,0}));
    connect(rotorA2.flange,rotor2.axis) annotation(Line(points = {{-268.8884788755796,-94.95679336559311},{-272.8884788755796,-94.95679336559311},{-272.8884788755796,-57.95679336559311},{-258.8884788755796,-57.95679336559311}},color = {0,0,0}));
    connect(pendulumAttachement_2.frame_a,rotor2.frame_b) annotation(Line(points = {{-66,-108},{-72,-108},{-72,-41.9567933655931},{-248.8884788755796,-41.9567933655931},{-248.8884788755796,-47.9567933655931}},color = {95,95,95}));
    connect(rotor2.frame_a,base.frame_b) annotation(Line(points = {{-248.8884788755796,-67.9567933655931},{-248.8884788755796,-73.9567933655931},{-151.77757277112315,-73.9567933655931},{-151.77757277112315,14},{-54.66666666666667,14},{-54.66666666666667,10.4}},color = {95,95,95}));
    annotation (
      versionDate="2014-02-04",
      Commands(file="Furuta.mos" "Simulate Furuta pendulum", file="Animate.mos"
          "Animate Furuta pendulum"),
      experiment(NumberOfIntervals=5000, StopTime=10),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}),     graphics),uses(Modelica(version = "3.2.3")));
end FurutaPendulum;
