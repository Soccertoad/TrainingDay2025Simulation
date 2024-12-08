package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.wrist.WristConstants.Length;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.arm.ArmConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class WristVisualizer {
  private final String key;
  private final LoggedMechanism2d panel;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d arm;
  private Pose3d latestPose;

  private static final double ElevatorXModifier = -Math.cos(Degrees.of(45).in(Radians));
  private static final double ElevatorZModifier = Math.sin(Degrees.of(45).in(Radians));

  public WristVisualizer(String key, Color color) {
    this.key = key;

    this.panel =
        new LoggedMechanism2d(
            Inches.of(100).in(Meters), Inches.of(100).in(Meters), new Color8Bit(Color.kBlack));
    this.root = panel.getRoot("Wrist", Inches.of(7.35).in(Meters), Inches.of(10).in(Meters));
    this.arm =
        root.append(
            new LoggedMechanismLigament2d("Wrist", Length.in(Meters), 0, 6, new Color8Bit(color)));

    Logger.recordOutput("Wrist/Mechanism2d/" + key, this.panel);
  }

  public void update(Angle position, Distance height, Angle armPosition) {
    Distance elevatorX = height.times(ElevatorXModifier);
    Distance elevatorZ = height.times(ElevatorZModifier);

    double wristXOffset =
        -elevatorX.in(Meters)
            + (Math.cos(armPosition.in(Radians))
                * ArmConstants.Length.minus(Inches.of(4)).in(Meters));
    double wristZOffset =
        elevatorZ.in(Meters)
            + (Math.sin(armPosition.in(Radians))
                * ArmConstants.Length.minus(Inches.of(1)).in(Meters));

    Pose3d wristPose =
        new Pose3d(
            new Translation3d(
                -wristXOffset,
                WristConstants.YPosition.in(Meters),
                wristZOffset + WristConstants.ZPosition.in(Meters)),
            new Rotation3d(0.0, position.in(Radians), 0.0));

    latestPose = wristPose;
    Logger.recordOutput("Wrist/Mechanism3d/" + key, wristPose);
  }

  public Pose3d getPose() {
    return latestPose;
  }
}
