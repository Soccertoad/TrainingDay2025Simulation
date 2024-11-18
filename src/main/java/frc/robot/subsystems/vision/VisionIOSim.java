package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionIOSim implements VisionIO{
    private static VisionSystemSim VisionSystemSim;

    private final PhotonPoseEstimator poseEstimator;
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    
    private Matrix<N3, N1> curStdDevs;

    public VisionIOSim(CameraConfig cameraConfig) {
        if (VisionSystemSim == null) {
            synchronized (VisionIOSim.class) {
                if (VisionSystemSim == null) {
                    VisionSystemSim = new VisionSystemSim("apriltag");
                    VisionSystemSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
                }
            }
        }
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

        inputs.name = camera.getName();
        Optional<EstimatedRobotPose> visionEst = getEstimatedGlobalPose();
        if (visionEst.isPresent()) {
            EstimatedRobotPose pose = visionEst.get();
            inputs.pose = pose.estimatedPose.toPose2d();
            inputs.timestamp = pose.timestampSeconds;
            inputs.curStdDevs = curStdDevs;

            int tagsSeen = pose.targetsUsed.size();
            List<Pose3d> tags = new ArrayList<>();

            for (int i = 0; i < tagsSeen; i++) {
                Optional<Pose3d> tagPose = VisionConstants.FieldLayout.getTagPose(pose.targetsUsed.get(i).fiducialId);
                if (tagPose.isPresent()) tags.add(tagPose.get());
            }

            inputs.tagPoses = tags.toArray(new Pose3d[tags.size()]);
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = poseEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.SingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.SingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.SingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.MultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
    
    public record CameraConfig(String name, Transform3d cameraPosition) {}
}
