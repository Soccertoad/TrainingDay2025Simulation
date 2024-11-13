package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.arm.ArmConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.wrist.WristConstants.Length;

public class WristVisualizer {
    private final String key;
    private final Mechanism2d panel;
    private final MechanismRoot2d root;
    private final MechanismLigament2d arm;

    private static final double ElevatorXModifier = -Math.cos(Degrees.of(45).in(Radians));
    private static final double ElevatorZModifier = Math.sin(Degrees.of(45).in(Radians));
  
    public WristVisualizer(String key, Color color) {
        this.key = key;

        this.panel = new Mechanism2d(Inches.of(100).in(Meters), Inches.of(100).in(Meters), new Color8Bit(Color.kBlack));
        this.root = panel.getRoot("Wrist", Inches.of(7.35).in(Meters), Inches.of(10).in(Meters));
        this.arm = root.append(
            new MechanismLigament2d(
                "Wrist",
                Length.in(Meters),
                0,
                6,
                new Color8Bit(color)
                )  
            );

        Logger.recordOutput("Wrist/Mechanism2d/" + key, this.panel);
    }

    public void update(Angle position, Distance height, Angle armPosition) {
        Distance elevatorX = height.times(ElevatorXModifier);
        Distance elevatorZ = height.times(ElevatorZModifier);

        double wristXOffset = -elevatorX.in(Meters) + (Math.cos(armPosition.in(Radians)) * ArmConstants.Length.minus(Inches.of(4)).in(Meters));
        double wristZOffset = elevatorZ.in(Meters) + (Math.sin(armPosition.in(Radians)) * ArmConstants.Length.minus(Inches.of(1)).in(Meters));

        Pose3d wristPose = new Pose3d(
            new Translation3d(
                -wristXOffset,
                WristConstants.YPosition.in(Meters),
                wristZOffset + WristConstants.ZPosition.in(Meters)
            ),
            new Rotation3d(0.0, position.in(Radians), 0.0)
        );

        Logger.recordOutput("Wrist/Mechanism3d/" + key, wristPose);
    }
  }
