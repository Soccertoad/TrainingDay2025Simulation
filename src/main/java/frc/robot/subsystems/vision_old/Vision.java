package frc.robot.subsystems.vision_old;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.VirtualSubsystem;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {

  private final VisionIO[] ios;
  private final VisionIOInputs_OLDAutoLogged[] inputs;
  private final double[] timestamps;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Consumer<VisionMeasurement> visionMeasurementConsumer;

  public record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> curStdDevs) {}
  ;

  public Vision(
      Supplier<Pose2d> robotPoseSupplier,
      Consumer<VisionMeasurement> visionMeasurementConsumer,
      VisionIO... io) {
    this.ios = io;
    this.inputs = new VisionIOInputs_OLDAutoLogged[io.length];
    this.robotPoseSupplier = robotPoseSupplier;
    this.visionMeasurementConsumer = visionMeasurementConsumer;
    this.timestamps = new double[io.length];
    Arrays.fill(this.timestamps, 0);
  }

  @Override
  public void periodic() {
    for (int index = 0; index < ios.length; index++) {
      var io = this.ios[index];
      var input = this.inputs[index];

      if (input == null) {
        input = new VisionIOInputs_OLDAutoLogged();
        this.inputs[index] = input;
      }

      io.updateInputs(input, robotPoseSupplier.get());
      Logger.processInputs("vision-" + input.name, input);

      if (input.timestamp > this.timestamps[index]) {
        this.timestamps[index] = input.timestamp;
        visionMeasurementConsumer.accept(
            new VisionMeasurement(input.pose, input.timestamp, input.curStdDevs));
      }
    }
  }
}
