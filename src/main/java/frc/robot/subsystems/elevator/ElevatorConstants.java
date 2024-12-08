package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.Gains;

public class ElevatorConstants {
  public static final CanDef LeaderProfile = CanDef.builder().id(2).bus(CanBus.Rio).build();

  public static final CanDef FollowerProfile = CanDef.builder().id(3).bus(CanBus.Rio).build();

  public static final Gains SimGains =
      Gains.builder().kS(0.0).kG(0.0).kV(0.0).kA(0.0).kP(0.3).kI(0.0).kD(0.0).build();

  public static final Gains TalonFXGains =
      Gains.builder().kS(0.0).kG(0.0).kV(0.0).kA(0.0).kP(0.0).kI(0.0).kD(0.0).build();

  public static final LinearVelocity MaxVelocity = FeetPerSecond.of(2);
  public static final LinearAcceleration MaxAcceleration = FeetPerSecondPerSecond.of(6);
  public static final double MaxJerk = 0.0;
  public static final Current TorqueCurrentLimit = Amps.of(120);
  public static final Current SupplyCurrentLimit = Amps.of(40);
  public static final Current ForwardTorqueLimit = Amps.of(80);
  public static final Current ReverseTorqueLimit = Amps.of(-80);

  public static final int NumMotors = 2;
  public static final double Gearing = 4;
  public static final Distance Height = Inches.of(24.719);
  public static final Mass Weight = Pounds.of(9.8);
  public static final Distance DrumRadius = Inches.of(2);
  public static final DCMotor Motors = DCMotor.getKrakenX60(NumMotors);
  public static final Distance MaximumHeight = Inches.of(52);
  public static final Distance MinimumHeight = Inches.of(0);
  public static final Distance StartingHeight = Inches.zero();

  public static final Angle ElevatorAngle = Degrees.of(45);
  public static final double XModifier = -Math.cos(ElevatorAngle.in(Radians));
  public static final double ZModifier = Math.sin(ElevatorAngle.in(Radians));
}
