package frc.robot.subsystems;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;

public record Initializer(Drive drive, Elevator elevator, Arm arm, Wrist wrist, Vision vision) {}
