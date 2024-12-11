package frc.robot.subsystems;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;
import static frc.robot.subsystems.vision.VisionConstants.camera3Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera2;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera3;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSim;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SimInitializer {
  public static Initializer initialize(SwerveDriveSimulation driveSim) {
    var drive =
        new Drive(
            new GyroIOSim(driveSim.getGyroSimulation()) {},
            new ModuleIOSim(driveSim.getModules()[0]),
            new ModuleIOSim(driveSim.getModules()[1]),
            new ModuleIOSim(driveSim.getModules()[2]),
            new ModuleIOSim(driveSim.getModules()[3]));

    return new Initializer(
        drive,
        new Elevator(new ElevatorIOSim()),
        new Arm(new ArmIOSim()),
        new Wrist(new WristIOSim()),
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                camera0Name, robotToCamera0, driveSim::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                camera1Name, robotToCamera1, driveSim::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                camera2Name, robotToCamera2, driveSim::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                camera3Name, robotToCamera3, driveSim::getSimulatedDriveTrainPose)));
  }
}
