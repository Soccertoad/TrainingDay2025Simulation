package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.util.Gains;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {

    private final Gains gains = Robot.isReal() ? ElevatorConstants.TalonFXGains : ElevatorConstants.SimGains;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private Distance setpoint = Inches.of(0.0);

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.io.setPID(gains.kP, gains.kI, gains.kD);
    }

    @Override
    public void periodic() {
        super.periodic();

        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.runSetpoint(this.setpoint);
        }

        RobotState.instance().updateElevatorPosition(this.inputs.position);
    }

    public Command setPosition(Distance position) {
        return runOnce(() -> this.setpoint = position);
    }

    public Command waitForGreaterThanPosition(Distance position) {
        return Commands.waitUntil(() -> this.inputs.position.gt(position));
    }

    public Command waitForLessThanPosition(Distance position) {
        return Commands.waitUntil(() -> this.inputs.position.lt(position));
    }
}
