package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.*;

public class ArmVisualizer {
    private final String key;
    private final Mechanism2d panel;
    private final MechanismRoot2d root;
    private final MechanismLigament2d arm;

    private static final double ElevatorXModifier = -Math.cos(Degrees.of(45).in(Radians));
    private static final double ElevatorZModifier = Math.sin(Degrees.of(45).in(Radians));
    private static final Distance ArmXModifier = Meters.of(0.07);
    private static final Distance ArmYModifier = Inches.of(3);
    private static final Distance ArmZModifier = Meters.of(0.377);
    private static final Angle ArmAngleModifier = Degrees.of(84);
  
    public ArmVisualizer(String key, Color color) {
        this.key = key;

        this.panel = new Mechanism2d(Inches.of(100).in(Meters), Inches.of(100).in(Meters), new Color8Bit(Color.kBlack));
        this.root = panel.getRoot("Arm", Inches.of(7.35).in(Meters), Inches.of(10).in(Meters));
        this.arm = root.append(
            new MechanismLigament2d(
                "Arm",
                Inches.of(24.719).in(Meters),
                0,
                6,
                new Color8Bit(color)
                )  
            );

        Logger.recordOutput("Arm/Mechanism2d/" + key, this.panel);
    }

    public void update(Angle position, Distance height) {
        arm.setAngle(position.in(Degrees));
        Logger.recordOutput("Arm/Mechanism2d/" + key, this.panel);

        Distance elevatorX = height.times(ElevatorXModifier);
        Distance elevatorZ = height.times(ElevatorZModifier);
        Distance armX = ArmXModifier.plus(elevatorX);
        Distance armY = ArmYModifier;
        Distance armZ = ArmZModifier.plus(elevatorZ);

        Pose3d arm3d = new Pose3d(armX, armY, armZ, new Rotation3d(0.0, position.minus(ArmAngleModifier).in(Radians), 0.0));
        Logger.recordOutput("Arm/Mechanism3d/" + key, arm3d);
    }
  }
