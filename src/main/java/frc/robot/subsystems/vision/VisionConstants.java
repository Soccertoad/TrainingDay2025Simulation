package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "front";
  public static String camera1Name = "back";
  public static String camera2Name = "left";
  public static String camera3Name = "right";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Meters.of(.2815),
          Meters.of(.2511),
          Meters.of(.2317),
          new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(0)));

  public static Transform3d robotToCamera1 =
      new Transform3d(
          Meters.of(.2815),
          Meters.of(.2511),
          Meters.of(.2317),
          new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(180)));

  public static Transform3d robotToCamera2 =
      new Transform3d(
          Meters.of(-0.09993),
          Meters.of(-0.316363),
          Meters.of(.2317),
          new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(90)));

  public static Transform3d robotToCamera3 =
      new Transform3d(
          Meters.of(-0.14949),
          Meters.of(0.291473),
          Meters.of(0.229238),
          new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-90)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
