package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
    
    @AutoLog
    class VisionIOInputs {
        public String name = "";
        public Pose2d pose = new Pose2d();
        public double timestamp = 0;
        public double ambiguity = 0;

        public Pose3d[] tagPoses = new Pose3d[0];
        public Matrix<N3, N1> curStdDevs = VisionConstants.SingleTagStdDevs;
    }

    record RawFiducial(
        int id,
        double ambiguity
    ) {};

    public void updateInputs(VisionIOInputs inputs, Pose2d robotPose);
}
