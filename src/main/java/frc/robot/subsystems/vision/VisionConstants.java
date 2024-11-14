package frc.robot.subsystems.vision;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {
    public static final SimCameraProperties SimCameraProperties = new SimCameraProperties() {{
        setCalibration(1280, 800, Rotation2d.fromDegrees(97.7));
        setCalibError(0.35, 0.10);
        setFPS(15);
        setAvgLatencyMs(20);
        setLatencyStdDevMs(5);
    }};

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
}
