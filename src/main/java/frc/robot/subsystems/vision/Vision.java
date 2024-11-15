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

    private final VisionIO[] ios;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Supplier<Pose2d> robotPoseSupplier;
    


    public Vision(Supplier<Pose2d> robotPoseSupplier, VisionIO ...io) {
        this.ios = io;
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        for (int index = 0; index < ios.length; index++) {
            var io = this.ios[index];
            var input = this.inputs[index];

            if (input == null) {
                input = new VisionIOInputsAutoLogged();
                this.inputs[index] = input;
            }

            io.updateInputs(input, robotPoseSupplier.get());

            Logger.processInputs("vision-" + input.name, input);

            List<Pose3d> tags = new ArrayList<>();

            for (int i = 0; i < input.tagsSeen.length; i++) {
                if (input.tagsSeen[i]) {
                    Optional<Pose3d> pose = VisionConstants.FieldLayout.getTagPose(i);
                    if (pose.isPresent()) tags.add(pose.get());
                }
            }

            Pose3d[] tagArray = tags.toArray(new Pose3d[tags.size()]);

            Logger.recordOutput("vision-" + input.name + "/TagPoses", tagArray);
        }
    }
    
}
