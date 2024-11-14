package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    
    @AutoLog
    class VisionIOInputs {
        public Pose2d pose = new Pose2d();
        public double timestampSeconds = 0;
        public double latency = 0;
        public int tagCount = 0;
        public boolean[] tagsSeen = new boolean[30];
    }

    record RawFiducial(
        int id,
        double ambiguity
    ) {};

    public void updateInputs(VisionIOInputs inputs, Pose2d robotPose);
}
