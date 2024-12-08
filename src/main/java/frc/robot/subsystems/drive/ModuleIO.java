package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public MutAngle drivePosition = Radians.mutable(0);
    public MutAngularVelocity driveVelocity = RadiansPerSecond.mutable(0);
    public MutVoltage driveAppliedVolts = Volts.mutable(0);
    public MutCurrent driveCurrent = Amps.mutable(0);

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public MutAngle turnAbsolutePosition = Radians.mutable(0);
    public MutAngle turnPosition = Radians.mutable(0);
    public MutAngularVelocity turnVelocity = RadiansPerSecond.mutable(0);
    public MutVoltage turnAppliedVolts = Volts.mutable(0);
    public MutCurrent turnCurrent = Amps.mutable(0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRads = new double[] {};
    public Rotation2d[] odometryTurnPositionsRads = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(Voltage volts) {}

  public default void setDriveOpenLoop(Current amps) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(Voltage volts) {}

  public default void setTurnOpenLoop(Current amps) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(AngularVelocity velocity) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Angle position) {}
}
