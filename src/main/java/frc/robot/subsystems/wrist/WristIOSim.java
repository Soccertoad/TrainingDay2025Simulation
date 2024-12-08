package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.wrist.WristConstants.Gearing;
import static frc.robot.subsystems.wrist.WristConstants.Length;
import static frc.robot.subsystems.wrist.WristConstants.MaxAcceleration;
import static frc.robot.subsystems.wrist.WristConstants.MaxVelocity;
import static frc.robot.subsystems.wrist.WristConstants.MaximumAngle;
import static frc.robot.subsystems.wrist.WristConstants.MinimumAngle;
import static frc.robot.subsystems.wrist.WristConstants.Motors;
import static frc.robot.subsystems.wrist.WristConstants.SimGains;
import static frc.robot.subsystems.wrist.WristConstants.StartingAngle;
import static frc.robot.subsystems.wrist.WristConstants.Weight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          Motors,
          Gearing,
          SingleJointedArmSim.estimateMOI(Length.in(Meters), Weight.in(Kilograms)),
          Length.in(Meters),
          MinimumAngle.in(Radians),
          MaximumAngle.in(Radians),
          true,
          StartingAngle.in(Radians));

  private final MutVoltage appliedVolts = Volts.mutable(0);
  private ArmFeedforward ff =
      new ArmFeedforward(SimGains.kS, SimGains.kG, SimGains.kV, SimGains.kA);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          SimGains.kP,
          SimGains.kI,
          SimGains.kD,
          new Constraints(
              MaxVelocity.in(DegreesPerSecond), MaxAcceleration.in(DegreesPerSecondPerSecond)));

  @Override
  public void updateInputs(WristIOInputs inputs) {
    sim.update(0.02);
    inputs.position.mut_replace(sim.getAngleRads(), Radians);
    inputs.velocity.mut_replace(sim.getVelocityRadPerSec(), RadiansPerSecond);

    inputs.appliedVoltsLeader.mut_replace(appliedVolts);
    inputs.appliedVoltsFollower.mut_replace(appliedVolts);

    inputs.supplyCurrentLeader.mut_replace(sim.getCurrentDrawAmps(), Amps);
    inputs.supplyCurrentFollower.mut_replace(sim.getCurrentDrawAmps(), Amps);

    inputs.torqueCurrentLeader.mut_replace(sim.getCurrentDrawAmps(), Amps);
    inputs.torqueCurrentFollower.mut_replace(sim.getCurrentDrawAmps(), Amps);

    inputs.temperatureLeader.mut_replace(0, Celsius);
    inputs.temperatureFollower.mut_replace(0, Celsius);

    inputs.setpointPosition.mut_replace(controller.getSetpoint().position, Degrees);
    inputs.setpointVelocity.mut_replace(controller.getSetpoint().velocity, DegreesPerSecond);
  }

  @Override
  public void runSetpoint(Angle angle) {
    Angle currentAngle = Radians.of(sim.getAngleRads());

    Angle setpointAngle = Degrees.of(controller.getSetpoint().position);
    AngularVelocity setpointVelocity = DegreesPerSecond.of(controller.getSetpoint().velocity);

    Voltage controllerVoltage =
        Volts.of(controller.calculate(currentAngle.in(Degrees), angle.in(Degrees)));
    Voltage feedForwardVoltage = ff.calculate(setpointAngle, setpointVelocity);

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    runVolts(effort);
  }

  @Override
  public void runVolts(Voltage volts) {
    double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
    appliedVolts.mut_replace(clampedEffort, Volts);
    sim.setInputVoltage(clampedEffort);
  }

  @Override
  public void setPID(double p, double i, double d) {
    controller.setPID(p, i, d);
  }

  @Override
  public void setFF(double kS, double kG, double kV, double kA) {
    this.ff = new ArmFeedforward(kS, kG, kV, kA);
  }

  @Override
  public void stop() {
    Angle currentAngle = Radians.of(sim.getAngleRads());
    controller.reset(currentAngle.in(Degrees));
    runVolts(Volts.of(0));
  }
}
