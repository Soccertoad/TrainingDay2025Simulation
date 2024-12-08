package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index, SwerveModuleConstants constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRads[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositionsRads[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(new Rotation2d(inputs.turnPosition));

    // Apply setpoints
    io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / constants.WheelRadius));
    io.setTurnPosition(state.angle.getMeasure());
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(Voltage volts) {
    io.setDriveOpenLoop(volts);
    io.setTurnPosition(Rotations.zero());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(Volts.zero());
    io.setTurnOpenLoop(Volts.zero());
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return new Rotation2d(inputs.turnPosition);
  }

  /** Returns the current drive position of the module in meters. */
  public Distance getPositionMeasure() {
    double position = inputs.drivePosition.in(Radians) * constants.WheelRadius;
    return Meters.of(position);
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public LinearVelocity getVelocityMeasure() {
    double velocity = inputs.driveVelocity.in(RadiansPerSecond) * constants.WheelRadius;
    return MetersPerSecond.of(velocity);
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeasure(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMeasure(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public Angle getWheelRadiusCharacterizationPosition() {
    return inputs.drivePosition;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public AngularVelocity getFFCharacterizationVelocity() {
    return inputs.driveVelocity;
  }
}
