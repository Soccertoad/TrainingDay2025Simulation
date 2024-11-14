package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.VirtualSubsystem;

public class Vision extends VirtualSubsystem {

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final Supplier<Pose2d> robotPoseSupplier;
    


    public Vision(VisionIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs, robotPoseSupplier.get());
        Logger.processInputs("vision", inputs);
        List<Pose3d> tags = new ArrayList<>();

        for (int i = 0; i < inputs.tagsSeen.length; i++) {
            if (inputs.tagsSeen[i]) {
                Optional<Pose3d> pose = VisionConstants.fieldLayout.getTagPose(i);
                if (pose.isPresent()) tags.add(pose.get());
            }
        }
        if (tags.size() > 0) Logger.recordOutput("vision/tagsSeen", tags.toArray(new Pose3d[tags.size()]));
        
    }
    
}
