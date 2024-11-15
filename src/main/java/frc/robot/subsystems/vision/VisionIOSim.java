package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOSim implements VisionIO{
    private static VisionSystemSim VisionSystemSim;

    private final PhotonPoseEstimator poseEstimator;
    private final CameraConfig config;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    

    public VisionIOSim(CameraConfig cameraConfig) {
        if (VisionSystemSim == null) {
            synchronized (VisionIOSim.class) {
                if (VisionSystemSim == null) {
                    VisionSystemSim = new VisionSystemSim("apriltag");
                    VisionSystemSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
                }
            }
        }
        this.config = cameraConfig;
        this.camera = new PhotonCamera(cameraConfig.name);
        this.cameraSim = new PhotonCameraSim(this.camera, VisionConstants.SimCameraProperties, 0, 5);
        VisionSystemSim.addCamera(this.cameraSim, cameraConfig.cameraPosition);
        this.poseEstimator = new PhotonPoseEstimator(
            VisionConstants.FieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraConfig.cameraPosition
        ) {{
            setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }};
    }


    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d robotPose) {
        VisionSystemSim.update(robotPose);
        Arrays.fill(inputs.tagsSeen, false);

        inputs.name = camera.getName();

        PhotonPipelineResult latest = camera.getLatestResult();
        Optional<MultiTargetPNPResult> pose = latest.multitagResult;

        if (pose.isPresent()) {
            Transform3d best = pose.get().estimatedPose.best.plus(config.cameraPosition.inverse());
            inputs.pose = new Pose3d(best.getTranslation(), best.getRotation()).toPose2d();
            pose.get().fiducialIDsUsed.forEach(id -> {
                Optional<Pose3d> idPose = VisionConstants.FieldLayout.getTagPose(id);
                if (idPose.isPresent()) inputs.tagsSeen[id] = true;
            });
        }

        inputs.latency = latest.metadata.getLatencyMillis() / 1000.0;
        inputs.tagCount = latest.getTargets().size();
        inputs.timestampSeconds = latest.getTimestampSeconds();
    }
    
    public record CameraConfig(String name, Transform3d cameraPosition) {}
}
