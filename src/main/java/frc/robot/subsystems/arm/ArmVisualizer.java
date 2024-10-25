package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
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

        double elevatorX = -height.in(Meters) * Math.cos(Units.degreesToRadians(45));
        double elevatorZ = height.in(Meters) * Math.sin(Units.degreesToRadians(45));
        double armX = -0.07 + elevatorX;
        double armY = Inches.of(3).in(Meters);
        double armZ = 0.377 + elevatorZ;

        Pose3d arm3d = new Pose3d(armX, armY, armZ, new Rotation3d(0.0, position.minus(Degrees.of(84)).in(Radians), 0.0));
        Logger.recordOutput("Arm/Mechanism3d/" + key, arm3d);
    }
  }
