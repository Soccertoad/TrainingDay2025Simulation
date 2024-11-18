package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.util.VirtualSubsystem;

public class GamepieceVisualizer extends VirtualSubsystem {
    public boolean isHeld = true;
    private Supplier<Pose3d> wristPoseSupplier = () -> new Pose3d();
    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    public GamepieceVisualizer(Supplier<Pose3d> wristPoseSupplier) {
        this.wristPoseSupplier = wristPoseSupplier;
        Logger.recordOutput("Gamepiece/HeldNotes", new Pose3d[] {});
    }

    public void update(boolean isHeld) {
        this.isHeld = isHeld;
    }

    public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    public Command shoot(LinearVelocity power) {
        return new ScheduleCommand(
            Commands.defer(
                () -> {
                    isHeld = false;
                    final Pose3d startPose = getPose();
                    final Pose3d endPose = startPose.transformBy(
                        new Transform3d(-2, 0, -1 + startPose.getZ(), new Rotation3d())
                    );

                    final double duration = startPose.getTranslation().getDistance(endPose.getTranslation()) / power.in(MetersPerSecond);
                    final Timer timer = new Timer();
                    timer.start();
                    return Commands.run(
                        () -> Logger.recordOutput("Gamepiece/EjectNote", new Pose3d[] {
                            startPose.interpolate(endPose, timer.get() / duration)   
                        })
                    ).until(() -> timer.hasElapsed(duration))
                    .finallyDo(() -> Logger.recordOutput("Gamepiece/EjectNote", new Pose3d[] {}));
                }, Set.of()
            )
        );
    }

    @Override
    public void periodic() {
        if (isHeld) {
            Logger.recordOutput("Gamepiece/HeldNotes", new Pose3d[] {getPose()});
        } else {
            Logger.recordOutput("Gamepiece/HeldNotes", new Pose3d[] {});
        }
    }

    private Pose3d getPose() {
        Pose3d robotPose = new Pose3d(robotPoseSupplier.get());
        Pose3d wristPose = wristPoseSupplier.get();
        if (robotPose != null && wristPose != null) {
            return robotPose.plus(
                new Transform3d(wristPose.getTranslation(), wristPose.getRotation())
            ).transformBy(new Transform3d(
                Inches.of(-9), 
                Inches.of(16), 
                Meters.of(0),
                new Rotation3d()
            ));
        } else {
            return new Pose3d();
        }
    }

}
