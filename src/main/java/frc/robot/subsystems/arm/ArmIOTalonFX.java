package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.arm.ArmConstants.FollowerProfile;
import static frc.robot.subsystems.arm.ArmConstants.ForwardTorqueLimit;
import static frc.robot.subsystems.arm.ArmConstants.Gearing;
import static frc.robot.subsystems.arm.ArmConstants.LeaderProfile;
import static frc.robot.subsystems.arm.ArmConstants.MaxAcceleration;
import static frc.robot.subsystems.arm.ArmConstants.MaxJerk;
import static frc.robot.subsystems.arm.ArmConstants.MaxVelocity;
import static frc.robot.subsystems.arm.ArmConstants.ReverseTorqueLimit;
import static frc.robot.subsystems.arm.ArmConstants.SupplyCurrentLimit;
import static frc.robot.subsystems.arm.ArmConstants.TalonFXGains;
import static frc.robot.subsystems.arm.ArmConstants.TorqueCurrentLimit;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX leader;
    private final TalonFX follower;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Double> setpointPositionSignal;
    private final StatusSignal<Double> setpointVelocitySignal;

    private final List<StatusSignal<Voltage>> appliedVoltageSignal;
    private final List<StatusSignal<Current>> supplyCurrentSignal;
    private final List<StatusSignal<Current>> torqueCurrentSignal;
    private final List<StatusSignal<Temperature>> temperatureSignal;

    private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public ArmIOTalonFX() {
        leader = new TalonFX(LeaderProfile.id(), LeaderProfile.bus());
        follower = new TalonFX(FollowerProfile.id(), FollowerProfile.bus());
        follower.setControl(new Follower(LeaderProfile.id(), true));
        
        config.Slot0.kP = TalonFXGains.kP;
        config.Slot0.kI = TalonFXGains.kI;
        config.Slot0.kD = TalonFXGains.kD;
        config.Slot0.kS = TalonFXGains.kS;
        config.Slot0.kG = TalonFXGains.kG;
        config.Slot0.kV = TalonFXGains.kV;
        config.Slot0.kA = TalonFXGains.kA;
        config.MotionMagic.MotionMagicJerk = MaxJerk;
        config.MotionMagic.MotionMagicAcceleration = MaxVelocity.in(DegreesPerSecond);
        config.MotionMagic.MotionMagicCruiseVelocity = MaxAcceleration.in(DegreesPerSecondPerSecond);

        config.TorqueCurrent.PeakForwardTorqueCurrent = ForwardTorqueLimit.in(Amps);
        config.TorqueCurrent.PeakReverseTorqueCurrent = ReverseTorqueLimit.in(Amps);
        config.CurrentLimits.StatorCurrentLimit = TorqueCurrentLimit.in(Amps);
        config.CurrentLimits.SupplyCurrentLimit = SupplyCurrentLimit.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = Gearing;
        leader.getConfigurator().apply(config, 1.0);

        positionSignal = leader.getPosition();
        velocitySignal = leader.getVelocity();
        setpointPositionSignal = leader.getClosedLoopReference();
        setpointVelocitySignal = leader.getClosedLoopReferenceSlope();

        appliedVoltageSignal = List.of(leader.getMotorVoltage(), follower.getMotorVoltage());
        supplyCurrentSignal = List.of(leader.getSupplyCurrent(), follower.getSupplyCurrent());
        torqueCurrentSignal = List.of(leader.getTorqueCurrent(), follower.getTorqueCurrent());
        temperatureSignal = List.of(leader.getDeviceTemp(), follower.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
        100,
            positionSignal,
            velocitySignal,
            appliedVoltageSignal.get(0),
            appliedVoltageSignal.get(1),
            supplyCurrentSignal.get(0),
            supplyCurrentSignal.get(1),
            torqueCurrentSignal.get(0),
            torqueCurrentSignal.get(1),
            temperatureSignal.get(0),
            temperatureSignal.get(1)
        );

        leader.optimizeBusUtilization(0, 1.0);
        follower.optimizeBusUtilization(0, 1.0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.leaderMotorConnected = BaseStatusSignal.refreshAll(
            positionSignal,
            velocitySignal,
            setpointPositionSignal,
            setpointVelocitySignal,
            appliedVoltageSignal.get(0),
            supplyCurrentSignal.get(0),
            torqueCurrentSignal.get(0),
            temperatureSignal.get(0))
        .isOK();

        inputs.followerMotorConnected = BaseStatusSignal.refreshAll(
            appliedVoltageSignal.get(1),
            supplyCurrentSignal.get(1),
            torqueCurrentSignal.get(1),
            temperatureSignal.get(1))
        .isOK();

        inputs.position.mut_replace(positionSignal.getValue());
        inputs.velocity.mut_replace(velocitySignal.getValue());

        inputs.appliedVoltsLeader.mut_replace(appliedVoltageSignal.get(0).getValue());
        inputs.appliedVoltsFollower.mut_replace(appliedVoltageSignal.get(1).getValue());

        inputs.supplyCurrentLeader.mut_replace(supplyCurrentSignal.get(0).getValue());
        inputs.supplyCurrentFollower.mut_replace(supplyCurrentSignal.get(1).getValue());

        inputs.torqueCurrentLeader.mut_replace(torqueCurrentSignal.get(0).getValue());
        inputs.torqueCurrentFollower.mut_replace(torqueCurrentSignal.get(1).getValue());

        inputs.temperatureLeader.mut_replace(temperatureSignal.get(0).getValue());
        inputs.temperatureFollower.mut_replace(temperatureSignal.get(1).getValue());

        inputs.setpointPosition.mut_replace(setpointPositionSignal.getValue(), Rotations);
        inputs.setpointVelocity.mut_replace(setpointVelocitySignal.getValue(), RotationsPerSecond);
    }

    @Override
    public void runSetpoint(Angle position) {
        leader.setControl(positionControl.withPosition(position));
    }

    @Override
    public void runVolts(Voltage volts) {
        leader.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setPID(double p, double i, double d) {
        config.Slot0.kP = p;
        config.Slot0.kI = i;
        config.Slot0.kD = d;
        leader.getConfigurator().apply(config, 0.01);
    }

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kG = kG;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        leader.getConfigurator().apply(config, 0.01);
    }
    
    @Override
    public void stop() {
        leader.setControl(new NeutralOut());
    }
}
