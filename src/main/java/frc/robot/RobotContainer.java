// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.DriveTrainSimulationConfig;
import frc.robot.util.SimulatedArena;
import frc.robot.util.SwerveDriveSimulation;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public static final SwerveDriveSimulation sim = new SwerveDriveSimulation(
        DriveTrainSimulationConfig.Default(),
        new Pose2d(3, 3, new Rotation2d()));

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Elevator elevator;
    private final Arm arm;

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;

        configureBindings();
        SimulatedArena.getInstance().addDriveTrainSimulation(sim);
        SimulatedArena.getInstance().resetFieldForAuto();
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                // forwardStraight.withVelocityX(0.5).withVelocityY(0)
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.y().whileTrue(drivetrain.applyRequest(() ->
            drive.withVelocityX(MaxSpeed).withVelocityY(0))
        );
        joystick.a().whileTrue(drivetrain.applyRequest(() ->
            drive.withVelocityX(-MaxSpeed).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // joystick.y().whileTrue(elevator.setPosition(Inches.of(25)).alongWith(arm.setPosition(Degrees.of(180))))
        //     .onFalse(elevator.setPosition(Inches.of(0)).alongWith(arm.setPosition(Degrees.of(70))));
        // joystick.a().whileTrue(elevator.setPosition(Inches.of(0)).alongWith(arm.setPosition(Degrees.of(70))));
        // joystick.x().whileTrue(elevator.setPosition(Inches.zero()).alongWith(arm.setPosition(Degrees.zero())));
        // joystick.b().whileTrue(elevator.setPosition(Inches.of(50)).alongWith(arm.setPosition(Degrees.of(0))))
        //     .onFalse(elevator.setPosition(Inches.of(0)).alongWith(arm.setPosition(Degrees.of(70))));
    }

    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        // return autoChooser.getSelected();
        return Commands.runOnce(() -> {});
    }
    
    public void updateFieldSimAndDisplay() {
        if (sim == null) return;
        Logger.recordOutput("FieldSimulation/RobotPosition", sim.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Notes",
                SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(Pose3d[]::new));
    }
}
