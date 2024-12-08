package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.Gains;

public class WristConstants {
  public static final CanDef LeaderProfile = CanDef.builder().id(0).bus(CanBus.Rio).build();

  public static final Gains SimGains =
      Gains.builder().kS(0.0).kG(0.0).kV(1.45).kA(0.0).kP(0.1).kI(0.0).kD(0.0).build();

  public static final Gains TalonFXGains =
      Gains.builder().kS(0.0).kG(0.0).kV(0.0).kA(0.0).kP(0.0).kI(0.0).kD(0.0).build();

  public static final AngularVelocity MaxVelocity = DegreesPerSecond.of(360);
  public static final AngularAcceleration MaxAcceleration = DegreesPerSecondPerSecond.of(360);
  public static final double MaxJerk = 0.0;
  public static final Current TorqueCurrentLimit = Amps.of(120);
  public static final Current SupplyCurrentLimit = Amps.of(40);
  public static final Current ForwardTorqueLimit = Amps.of(80);
  public static final Current ReverseTorqueLimit = Amps.of(-80);

  public static final int NumMotors = 1;
  public static final double Gearing = 75;
  public static final Distance Length = Inches.of(9.4);
  public static final Mass Weight = Pounds.of(4);
  public static final DCMotor Motors = DCMotor.getKrakenX60(NumMotors);
  public static final Angle MaximumAngle = Degrees.of(360);
  public static final Angle MinimumAngle = Degrees.of(-360);
  public static final Angle StartingAngle = Degrees.zero();

  public static final Distance XPosition = Inches.of(2.25);
  public static final Distance YPosition = Inches.of(-15);
  public static final Distance ZPosition = Inches.of(20.5);
  public static final Angle PitchModifier = Degrees.of(-40);
}
