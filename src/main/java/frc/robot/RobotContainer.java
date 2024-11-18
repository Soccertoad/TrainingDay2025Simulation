// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.wrist.Wrist;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Vision vision = new Vision(
        () -> this.drivetrain.getState().Pose,
        (visionMeasurement) -> {
            this.drivetrain.addVisionMeasurement(visionMeasurement.pose(), Utils.fpgaToCurrentTime(visionMeasurement.timestamp()), visionMeasurement.curStdDevs());
        },
        new VisionIOSim(
            new VisionIOSim.CameraConfig(
                "front",
                new Transform3d(
                    Meters.of(.2815),
                    Meters.of(.2511),
                    Meters.of(.2317),
                    new Rotation3d(
                        Degrees.of(0), 
                        Degrees.of(-20), 
                        Degrees.of(0)
                    )
                )
            )
        ),
        new VisionIOSim(
            new VisionIOSim.CameraConfig(
                "back",
                new Transform3d(
                    Meters.of(.2815),
                    Meters.of(.2511),
                    Meters.of(.2317),
                    new Rotation3d(
                        Degrees.of(0), 
                        Degrees.of(-20), 
                        Degrees.of(180)
                    )
                )
            )
        ),
        new VisionIOSim(
            new VisionIOSim.CameraConfig(
                "left",
                new Transform3d(
                    Meters.of(-0.09993),
                    Meters.of(-0.316363),
                    Meters.of(.2317),
                    new Rotation3d(
                        Degrees.of(0), 
                        Degrees.of(-20), 
                        Degrees.of(90)
                    )
                )
            )
        ),
        new VisionIOSim(
            new VisionIOSim.CameraConfig(
                "right",
                new Transform3d(
                    Meters.of(-0.14949),
                    Meters.of(0.291473),
                    Meters.of(0.229238),
                    new Rotation3d(
                        Degrees.of(0), 
                        Degrees.of(-20), 
                        Degrees.of(-90)
                    )
                )
            )
        )
    );

    private final Elevator elevator;
    private final Arm arm;
    private final Wrist wrist;

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer(Elevator elevator, Arm arm, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;

        configureBindings();
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
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

        joystick.y().whileTrue(
            elevator.setPosition(Inches.of(25)).alongWith(arm.setPosition(Degrees.of(180))).alongWith(wrist.setPosition(Degrees.of(100))))
            .onFalse(elevator.setPosition(Inches.of(0)).alongWith(arm.setPosition(Degrees.of(70))).alongWith(wrist.setPosition(Degrees.of(0))));
        joystick.a().whileTrue(elevator.setPosition(Inches.of(0)).alongWith(arm.setPosition(Degrees.of(70))));
        joystick.x().whileTrue(elevator.setPosition(Inches.zero()).alongWith(arm.setPosition(Degrees.zero())));
        joystick.b().whileTrue(elevator.setPosition(Inches.of(50)).alongWith(arm.setPosition(Degrees.of(0))).alongWith(wrist.setPosition(Degrees.of(-60))))
            .onFalse(elevator.setPosition(Inches.of(0)).alongWith(arm.setPosition(Degrees.of(70))).alongWith(wrist.setPosition(Degrees.of(0))));

        
    }

    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        // return autoChooser.getSelected();
        return Commands.runOnce(() -> {});
    }
}
