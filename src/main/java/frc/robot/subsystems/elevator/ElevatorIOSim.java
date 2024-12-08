package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.DrumRadius;
import static frc.robot.subsystems.elevator.ElevatorConstants.Gearing;
import static frc.robot.subsystems.elevator.ElevatorConstants.MaximumHeight;
import static frc.robot.subsystems.elevator.ElevatorConstants.MinimumHeight;
import static frc.robot.subsystems.elevator.ElevatorConstants.Motors;
import static frc.robot.subsystems.elevator.ElevatorConstants.SimGains;
import static frc.robot.subsystems.elevator.ElevatorConstants.StartingHeight;
import static frc.robot.subsystems.elevator.ElevatorConstants.Weight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim =
      new ElevatorSim(
          Motors,
          Gearing,
          Weight.in(Kilograms),
          DrumRadius.in(Meters),
          MinimumHeight.in(Meters),
          MaximumHeight.in(Meters),
          true,
          StartingHeight.in(Meters));

  private final MutVoltage appliedVolts = Volts.mutable(0);
  private final PIDController controller = new PIDController(SimGains.kP, SimGains.kI, SimGains.kD);
  private final ElevatorFeedforward ff =
      new ElevatorFeedforward(SimGains.kS, SimGains.kV, SimGains.kA);

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);
    inputs.position.mut_replace(sim.getPositionMeters(), Meters);
    inputs.velocity.mut_replace(sim.getVelocityMetersPerSecond(), MetersPerSecond);

    inputs.appliedVoltsLeader.mut_replace(appliedVolts);
    inputs.appliedVoltsFollower.mut_replace(appliedVolts);

    inputs.supplyCurrentLeader.mut_replace(sim.getCurrentDrawAmps(), Amps);
    inputs.supplyCurrentFollower.mut_replace(sim.getCurrentDrawAmps(), Amps);

    inputs.torqueCurrentLeader.mut_replace(sim.getCurrentDrawAmps(), Amps);
    inputs.torqueCurrentFollower.mut_replace(sim.getCurrentDrawAmps(), Amps);

    inputs.temperatureLeader.mut_replace(0, Celsius);
    inputs.temperatureFollower.mut_replace(0, Celsius);

    inputs.setpointPosition.mut_replace(controller.getSetpoint(), Meters);
    inputs.setpointVelocity.mut_replace(0, MetersPerSecond);
  }

  @Override
  public void runSetpoint(Distance position) {
    Distance currentHeight = Meters.of(sim.getPositionMeters());
    LinearVelocity currentVelocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());

    Voltage controllerVoltage =
        Volts.of(controller.calculate(currentHeight.in(Inches), position.in(Inches)));
    Voltage feedForwardVoltage = ff.calculate(currentVelocity);

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
  public void stop() {
    runVolts(Volts.of(0));
  }
}
