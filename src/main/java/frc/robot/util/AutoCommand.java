package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.json.simple.parser.ParseException;

public abstract class AutoCommand extends SequentialCommandGroup {
  public abstract List<Pose2d> getAllPathPoses(boolean isRedAlliance);

  public abstract Pose2d getStartingPose(boolean isRedAlliance);

  public final Optional<PathPlannerPath> loadChoreoAuto(String name) {
    try {
      return Optional.ofNullable(PathPlannerPath.fromChoreoTrajectory(name));
    } catch (FileVersionException | IOException | ParseException e) {
      return Optional.empty();
    }
  }
}
