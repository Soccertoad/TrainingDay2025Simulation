package frc.robot.subsystems;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;

public class DefaultInitializer {
  public static Initializer initialize() {
    var drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});

    return new Initializer(
        drive,
        new Elevator(new ElevatorIO() {}),
        new Arm(new ArmIO() {}),
        new Wrist(new WristIO() {}),
        new Vision(
            drive::addVisionMeasurement,
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {}));
  }
}
