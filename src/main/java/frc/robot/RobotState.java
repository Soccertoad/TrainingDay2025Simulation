package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmVisualizer;
import frc.robot.subsystems.elevator.ElevatorVisualizer;
import frc.robot.subsystems.wrist.WristVisualizer;
import java.util.function.Supplier;

public class RobotState {
  private static RobotState instance;

  private MutDistance elevatorPosition;
  private MutAngle armPosition;
  private MutAngle wristPosition;

  private ArmVisualizer armVisualizer = new ArmVisualizer("measured", Color.kGreen);
  private ElevatorVisualizer elevatorVisualizer = new ElevatorVisualizer("measured", Color.kGreen);
  private WristVisualizer wristVisualizer = new WristVisualizer("measured", Color.kGreen);
  private GamepieceVisualizer gamepieceVisualizer =
      new GamepieceVisualizer(wristVisualizer::getPose);

  private RobotState() {
    elevatorPosition = Inches.mutable(0);
    armPosition = Degrees.mutable(0);
    wristPosition = Degrees.mutable(0);
  }

  public static RobotState instance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
    gamepieceVisualizer.setRobotPoseSupplier(robotPoseSupplier);
  }

  public Command shoot(LinearVelocity power) {
    return gamepieceVisualizer.shoot(power);
  }

  public Distance getElevatorPosition() {
    return elevatorPosition;
  }

  public void updateElevatorPosition(Distance position) {
    elevatorPosition.mut_replace(position);
    elevatorVisualizer.update(elevatorPosition);
    wristVisualizer.update(wristPosition, elevatorPosition, armPosition);
  }

  public Angle getArmPosition() {
    return armPosition;
  }

  public void updateArmAngle(Angle position) {
    armPosition.mut_replace(position);
    armVisualizer.update(armPosition, elevatorPosition);
    wristVisualizer.update(wristPosition, elevatorPosition, armPosition);
  }

  public Angle getWristPosition() {
    return wristPosition;
  }

  public void updateWristAngle(Angle position) {
    wristPosition.mut_replace(position);
    wristVisualizer.update(wristPosition, elevatorPosition, armPosition);
  }

  public void updateHeldGamepiece(boolean isHeld) {
    gamepieceVisualizer.update(isHeld);
  }
}
