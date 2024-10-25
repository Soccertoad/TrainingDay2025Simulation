package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import static edu.wpi.first.units.Units.*;

public class Arm extends SubsystemBase {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final ArmVisualizer measuredVisualizer;
    private final ArmVisualizer setpointVisualizer;
    private final ArmVisualizer goalVisualizer;

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;

    private Angle setpoint = Degrees.of(0.0);

    public Arm(ArmIO io) {
        this.io = io;
        this.io.setPID(.15, 0, 0);
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();

        this.measuredVisualizer = new ArmVisualizer("measured", Color.kWhite);
        this.setpointVisualizer = new ArmVisualizer("setpoint", Color.kBlue);
        this.goalVisualizer = new ArmVisualizer("goal", Color.kGreen);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.updateInputs(inputs);
            Logger.processInputs("Arm", inputs);
            this.io.runSetpoint(this.setpoint);
        }

        this.measuredVisualizer.update(this.inputs.position, actual.getElevatorPosition());
        this.setpointVisualizer.update(this.inputs.setpointPosition, target.getElevatorPosition());
        this.goalVisualizer.update(this.setpoint, goal.getElevatorPosition());

        actual.updateArmAngle(this.inputs.position);
        target.updateArmAngle(this.inputs.setpointPosition);
        goal.updateArmAngle(this.setpoint);
    }

    public Command setPosition(Angle position) {
        return runOnce(() -> this.setpoint = position);
    }
}
