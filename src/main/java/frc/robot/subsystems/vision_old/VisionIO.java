package frc.robot.subsystems.vision_old;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  class VisionIOInputs_OLD {
    public String name = "";
    public Pose2d pose = new Pose2d();
    public double timestamp = 0;
    public double ambiguity = 0;

    public Pose3d[] tagPoses = new Pose3d[0];
    public Matrix<N3, N1> curStdDevs = VisionConstants.SingleTagStdDevs;
  }

  record RawFiducial(int id, double ambiguity) {}
  ;

  public default void updateInputs(VisionIOInputs_OLD inputs, Pose2d robotPose) {}
  ;
}
