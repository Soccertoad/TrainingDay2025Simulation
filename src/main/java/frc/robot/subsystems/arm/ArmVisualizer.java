package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.Length;
import static frc.robot.subsystems.arm.ArmConstants.PitchModifier;
import static frc.robot.subsystems.arm.ArmConstants.XPosition;
import static frc.robot.subsystems.arm.ArmConstants.YPosition;
import static frc.robot.subsystems.arm.ArmConstants.ZPosition;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmVisualizer {
  private final String key;
  private final LoggedMechanism2d panel;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d arm;

  public static final double ElevatorXModifier = -Math.cos(Degrees.of(45).in(Radians));
  public static final double ElevatorZModifier = Math.sin(Degrees.of(45).in(Radians));

  public ArmVisualizer(String key, Color color) {
    this.key = key;

    this.panel =
        new LoggedMechanism2d(
            Inches.of(100).in(Meters), Inches.of(100).in(Meters), new Color8Bit(Color.kBlack));
    this.root = panel.getRoot("Arm", Inches.of(7.35).in(Meters), Inches.of(10).in(Meters));
    this.arm =
        root.append(
            new LoggedMechanismLigament2d("Arm", Length.in(Meters), 0, 6, new Color8Bit(color)));

    Logger.recordOutput("Arm/Mechanism2d/" + key, this.panel);
  }

  public void update(Angle position, Distance height) {
    arm.setAngle(position.in(Degrees));
    Logger.recordOutput("Arm/Mechanism2d/" + key, this.panel);

    Distance elevatorX = height.times(ElevatorXModifier);
    Distance elevatorZ = height.times(ElevatorZModifier);
    Distance armX = XPosition.plus(elevatorX);
    Distance armY = YPosition;
    Distance armZ = ZPosition.plus(elevatorZ);

    Pose3d arm3d =
        new Pose3d(
            armX, armY, armZ, new Rotation3d(0.0, position.minus(PitchModifier).in(Radians), 0.0));
    Logger.recordOutput("Arm/Mechanism3d/" + key, arm3d);
  }
}
