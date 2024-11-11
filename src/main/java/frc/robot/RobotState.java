package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.arm.ArmVisualizer;
import frc.robot.subsystems.elevator.ElevatorVisualizer;

import static edu.wpi.first.units.Units.*;

public class RobotState {
    private static RobotState instance;

    private MutDistance elevatorPosition;
    private MutAngle armPosition;

    private ArmVisualizer armVisualizer = new ArmVisualizer("measured", Color.kGreen);
    private ElevatorVisualizer elevatorVisualizer = new ElevatorVisualizer("measured", Color.kGreen);

    private RobotState() {
        elevatorPosition = Inches.mutable(0);
        armPosition = Degrees.mutable(0);
    }

    public static RobotState instance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    public Distance getElevatorPosition() {
        return elevatorPosition;
    }

    public void updateElevatorPosition(Distance position) {
        elevatorPosition.mut_replace(position);
        elevatorVisualizer.update(elevatorPosition);
    }

    public Angle getArmPosition() {
        return armPosition;
    }
    
    public void updateArmAngle(Angle position) {
        armPosition.mut_replace(position);
        armVisualizer.update(armPosition, elevatorPosition);
    }
}