package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionIOSim implements VisionIO{
    private final VisionSystemSim visionSystemSim;

    private final PhotonPoseEstimator[] poseEstimators;
    private final PhotonCamera[] cameras;
    private final PhotonCameraSim[] cameraSims;
    

    public VisionIOSim(CameraConfig ...cameraConfigs) {
        this.visionSystemSim = new VisionSystemSim("apriltag");
        this.cameras = new PhotonCamera[cameraConfigs.length];
        this.cameraSims = new PhotonCameraSim[cameraConfigs.length];
        this.poseEstimators = new PhotonPoseEstimator[cameraConfigs.length];

        for(int index = 0; index < cameraConfigs.length; index++) {
            this.cameras[index] = new PhotonCamera(cameraConfigs[index].name);
            this.cameraSims[index] = new PhotonCameraSim(this.cameras[index], VisionConstants.SimCameraProperties);
            this.visionSystemSim.addCamera(this.cameraSims[index], cameraConfigs[index].cameraPosition);
            this.poseEstimators[index] = new PhotonPoseEstimator(
                VisionConstants.fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraConfigs[index].cameraPosition
            ) {{
                setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            }};
        }

        this.visionSystemSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
    }


    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d robotPose) {
        this.visionSystemSim.update(robotPose);
        Arrays.fill(inputs.tagsSeen, false);

        for(int index = 0; index < cameraSims.length; index++) {
            PhotonPipelineResult latest = cameras[index].getLatestResult();
            Optional<MultiTargetPNPResult> pose = latest.multitagResult;

            if (pose.isPresent()) {
                Transform3d best = pose.get().estimatedPose.best;
                inputs.pose = new Pose3d(best.getTranslation(), best.getRotation()).toPose2d();
                pose.get().fiducialIDsUsed.forEach(id -> {
                    Optional<Pose3d> idPose = VisionConstants.fieldLayout.getTagPose(id);
                    if (idPose.isPresent()) inputs.tagsSeen[id] = true;
                });
            }

            inputs.latency = latest.metadata.getLatencyMillis() / 1000.0;
            inputs.tagCount = latest.getTargets().size();

            
            // inputs.tags[index] = new int[inputs.tagCount[index]];
            // inputs.tagAmbiguities[index] = new double[inputs.tagCount[index]];

            // PhotonTrackedTarget target;
            // for (int i = 0; i < inputs.tagCount[index]; i++) {
            //     target = targets.get(i);
            //     inputs.tags[index][i] = target.fiducialId;
            //     inputs.tagAmbiguities[index][i] = target.poseAmbiguity;
            // }

        }
    }
    
    public record CameraConfig(String name, Transform3d cameraPosition) {}
}
