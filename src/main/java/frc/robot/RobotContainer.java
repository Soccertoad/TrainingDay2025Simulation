// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.processors.NamedCommandProcessor;
import frc.robot.autos.Amp;
import frc.robot.autos.AmpThree;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DefaultInitializer;
import frc.robot.subsystems.Initializer;
import frc.robot.subsystems.RealInitializer;
import frc.robot.subsystems.SimInitializer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AutoCommand;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  private SwerveDriveSimulation driveSim = null;
  private final CommandXboxController joystick = new CommandXboxController(0);

  private final Vision vision;

  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Wrist wrist;

  private final Field2d autoPreview = new Field2d();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    Initializer initalizer;

    switch (Constants.currentMode) {
      case REAL:
        initalizer = RealInitializer.initialize();
        break;

      case SIM:
        driveSim =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        initalizer = SimInitializer.initialize(driveSim);
        break;

      default:
        initalizer = DefaultInitializer.initialize();
        break;
    }

    this.drive = initalizer.drive();
    this.elevator = initalizer.elevator();
    this.arm = initalizer.arm();
    this.wrist = initalizer.wrist();
    this.vision = initalizer.vision();

    configureBindings();
    RobotState.instance().setRobotPoseSupplier(() -> drive.getPose());

    NamedCommandProcessor.registerDependency(Elevator.class, elevator);
    NamedCommandProcessor.registerDependency(Arm.class, arm);
    NamedCommandProcessor.registerDependency(Wrist.class, wrist);
    NamedCommandProcessor.processClass(AmpThree.class);

    SmartDashboard.putData("Auto Preview", autoPreview);

    autoChooser = AutoBuilder.buildAutoChooser("None");
    autoChooser.addOption("Amp Three", new Amp(arm, wrist, elevator));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.onChange(
        autoCommand -> {
          if (autoCommand instanceof AutoCommand auto) {
            boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

            autoPreview.getObject("path").setPoses(auto.getAllPathPoses(isRed));
            drive.setPose(auto.getStartingPose(isRed));
          } else {
            autoPreview.getObject("path").setPoses(new Pose2d[0]);
          }
        });

    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("Auto/ActivePath", poses.toArray(new Pose2d[0])));

    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Auto/TargetPathPose", pose));

    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX()));

    joystick
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -joystick.getLeftY(),
                () -> -joystick.getLeftX(),
                () -> new Rotation2d()));

    joystick
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    joystick.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    joystick
        .y()
        .whileTrue(
            elevator
                .setPosition(Inches.of(25))
                .alongWith(arm.setPosition(Degrees.of(180)))
                .alongWith(wrist.setPosition(Degrees.of(100))))
        .onFalse(
            elevator
                .setPosition(Inches.of(0))
                .alongWith(arm.setPosition(Degrees.of(70)))
                .alongWith(wrist.setPosition(Degrees.of(0))));
    joystick
        .a()
        .whileTrue(elevator.setPosition(Inches.of(0)).alongWith(arm.setPosition(Degrees.of(70))));
    joystick
        .x()
        .whileTrue(elevator.setPosition(Inches.zero()).alongWith(arm.setPosition(Degrees.zero())));
    joystick
        .b()
        .whileTrue(
            elevator
                .setPosition(Inches.of(50))
                .alongWith(arm.setPosition(Degrees.of(0)))
                .alongWith(wrist.setPosition(Degrees.of(-60))))
        .onFalse(
            elevator
                .setPosition(Inches.of(0))
                .alongWith(arm.setPosition(Degrees.of(70)))
                .alongWith(wrist.setPosition(Degrees.of(0))));

    joystick
        .rightBumper()
        .toggleOnTrue(RobotState.instance().shoot(MetersPerSecond.of(5)))
        .toggleOnFalse(Commands.run(() -> RobotState.instance().updateHeldGamepiece(true)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSim.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Notes",
        SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
  }
}
