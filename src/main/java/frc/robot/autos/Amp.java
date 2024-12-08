package frc.robot.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AutoCommand;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Amp extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;

  public Amp(Arm arm, Wrist wrist, Elevator elevator) {
    try {
      path1 = PathPlannerPath.fromPathFile("AmpThree.1");
      path2 = PathPlannerPath.fromPathFile("AmpThree.2");

      addCommands(
          Commands.sequence(
              AutoBuilder.followPath(path1)
                  .deadlineFor(
                      Commands.sequence(
                          arm.setPosition(Degrees.of(60))
                              .alongWith(wrist.setPosition(Degrees.of(35))),
                          arm.untilGreaterThan(Degrees.of(55))
                              .andThen(RobotState.instance().shoot(MetersPerSecond.of(5))))),
              elevator
                  .setPosition(Inches.of(0))
                  .alongWith(arm.setPosition(Degrees.of(180)))
                  .alongWith(wrist.setPosition(Degrees.of(135))),
              Commands.waitSeconds(0.5),
              AutoBuilder.followPath(path2),
              Commands.sequence(
                  arm.setPosition(Degrees.of(60)).alongWith(wrist.setPosition(Degrees.of(20))),
                  arm.untilLessThan(Degrees.of(62))
                      .andThen(RobotState.instance().shoot(MetersPerSecond.of(5))))));

    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public List<Pose2d> getAllPathPoses(boolean isRedAlliance) {
    return Stream.of((isRedAlliance ? path1.flipPath() : path1).getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getStartingPose(boolean isRedAlliance) {
    PathPlannerPath path = isRedAlliance ? path1.flipPath() : path1;
    Rotation2d rotation = new Rotation2d();

    if (path.getIdealStartingState() != null) {
      rotation = path.getIdealStartingState().rotation();
    }

    return new Pose2d(path.getPoint(0).position, rotation);
  }
}
