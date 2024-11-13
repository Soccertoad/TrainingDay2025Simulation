package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.util.SwerveDriveSimulation;
import frc.robot.util.SwerveModuleSimulation;

import static edu.wpi.first.units.Units.*;

public class MapleSimSwerveDrivetrain extends SimSwerveDrivetrain {
    private final SwerveDriveSimulation sim;
    private final SwerveModuleSimulation[] modules;
    private final SwerveModuleConstants[] moduleConstants;
    private final SwerveDriveKinematics kinematics;
    private Rotation2d lastAngle = new Rotation2d();
    public MapleSimSwerveDrivetrain(Translation2d[] wheelLocations, Pigeon2SimState pigeonSim, SwerveModuleConstants... moduleConstants) {
        super(wheelLocations, pigeonSim, moduleConstants);
        this.moduleConstants = moduleConstants;

        kinematics = new SwerveDriveKinematics(wheelLocations);
        this.sim = RobotContainer.sim;
        modules = this.sim.getModules();
    }

    @Override
    public void update(double dtSeconds, double supplyVoltage, SwerveModule... modulesToApply) {
        if (modulesToApply.length != m_modules.length) return;
        SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
        
        for (int i = 0; i < modulesToApply.length; ++i) {
            TalonFXSimState driveMotor = modulesToApply[i].getDriveMotor().getSimState();
            TalonFXSimState steerMotor = modulesToApply[i].getSteerMotor().getSimState();
            CANcoderSimState cancoder = modulesToApply[i].getCANcoder().getSimState();

            driveMotor.Orientation = moduleConstants[i].DriveMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            steerMotor.Orientation = moduleConstants[i].SteerMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;

            driveMotor.setSupplyVoltage(supplyVoltage);
            steerMotor.setSupplyVoltage(supplyVoltage);
            cancoder.setSupplyVoltage(supplyVoltage);

            // if (DriverStation.isEnabled()) {
            //     modules[i].requestDriveVoltageOut(addFriction(driveMotor.getMotorVoltage(), modules[i].DRIVE_FRICTION_VOLTAGE));
            //     modules[i].requestSteerVoltageOut(addFriction(steerMotor.getMotorVoltage(), modules[i].DRIVE_FRICTION_VOLTAGE));
            // }
            modules[i].requestDriveVoltageOut(driveMotor.getMotorVoltage());
            modules[i].requestSteerVoltageOut(steerMotor.getMotorVoltage());

            Logger.recordOutput("Module/" + i + "/Drive/MotorVoltage", driveMotor.getMotorVoltage());
            Logger.recordOutput("Module/" + i + "/Steer/MotorVoltage", steerMotor.getMotorVoltage());
            Logger.recordOutput("Module/" + i + "/Drive/UngearedPosition", modules[i].getDriveEncoderUnGearedPositionRad());
            Logger.recordOutput("Module/" + i + "/Drive/UngearedSpeed", modules[i].getDriveEncoderUnGearedSpeedRadPerSec());
            Logger.recordOutput("Module/" + i + "/Drive/FinalSpeed", modules[i].getDriveWheelFinalSpeedRadPerSec());
            Logger.recordOutput("Module/" + i + "/Steer/RelativePosition", modules[i].getSteerRelativeEncoderPositionRad());
            Logger.recordOutput("Module/" + i + "/Steer/RelativeSpeed", modules[i].getSteerRelativeEncoderSpeedRadPerSec() / moduleConstants[i].SteerMotorGearRatio);
            Logger.recordOutput("Module/" + i + "/Steer/AbsoluteSpeed", modules[i].getSteerAbsoluteEncoderSpeedRadPerSec());
            Logger.recordOutput("Module/" + i + "/CANcoder/Position", modules[i].getSteerRelativeEncoderPositionRad() / moduleConstants[i].SteerMotorGearRatio);
            Logger.recordOutput("Module/" + i + "/CANcoder/Velocity", modules[i].getSteerRelativeEncoderSpeedRadPerSec() / moduleConstants[i].SteerMotorGearRatio);

            driveMotor.setRawRotorPosition(Radians.of(modules[i].getDriveEncoderUnGearedPositionRad()).in(Rotations));
            driveMotor.setRotorVelocity(RadiansPerSecond.of(modules[i].getDriveEncoderUnGearedSpeedRadPerSec()).in(RotationsPerSecond));

            Rotation2d steerPos = Rotation2d.fromRadians(modules[i].getSteerRelativeEncoderPositionRad());

            steerMotor.setRawRotorPosition(steerPos.getRotations());
            steerMotor.setRotorVelocity(RadiansPerSecond.of(modules[i].getSteerRelativeEncoderSpeedRadPerSec()).in(RotationsPerSecond));

            cancoder.setRawPosition(modules[i].getSteerAbsoluteFacing().getRotations());
            cancoder.setVelocity(RadiansPerSecond.of(modules[i].getSteerAbsoluteEncoderSpeedRadPerSec()).in(RotationsPerSecond));

            states[i] = modulesToApply[i].getCurrentState();
        }

        Logger.recordOutput("Modules", states);

        double angularVelRadPerSec = kinematics.toChassisSpeeds(states).omegaRadiansPerSecond;
        lastAngle = lastAngle.plus(Rotation2d.fromRadians(angularVelRadPerSec * dtSeconds));
        // this.sim.getGyroSimulation().setRotation(Rotation2d.fromDegrees(lastAngle.getDegrees()));

        Logger.recordOutput("Gryo/Angle", this.sim.getGyroSimulation().getGyroReading().getDegrees());
        m_pigeonSim.setRawYaw(this.sim.getGyroSimulation().getGyroReading().getDegrees());
        // m_pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(angularVelRadPerSec));

    }

    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }
}
