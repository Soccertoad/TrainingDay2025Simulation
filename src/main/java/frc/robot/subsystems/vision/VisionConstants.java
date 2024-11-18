package frc.robot.subsystems.vision;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    public static final SimCameraProperties SimCameraProperties = new SimCameraProperties() {{
        setCalibration(1280, 800, Rotation2d.fromDegrees(97.7));
        setCalibError(0.35, 0.10);
        setFPS(15);
        setAvgLatencyMs(20);
        setLatencyStdDevMs(5);
    }};

    public static final AprilTagFieldLayout FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
