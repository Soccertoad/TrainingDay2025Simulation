package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.util.Gains;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

public class Arm extends SubsystemBase {

    private final Gains gains = Robot.isReal() ? ArmConstants.TalonFXGains : ArmConstants.SimGains;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", gains.kP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", gains.kI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", gains.kD);
    private final LoggedTunableNumber kS =
        new LoggedTunableNumber("Arm/Gains/kS", gains.kS);
    private final LoggedTunableNumber kV =
        new LoggedTunableNumber("Arm/Gains/kV", gains.kV);
    private final LoggedTunableNumber kA =
        new LoggedTunableNumber("Arm/Gains/kA", gains.kA);
    private final LoggedTunableNumber kG =
        new LoggedTunableNumber("Arm/Gains/kG", gains.kG);

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private Angle setpoint = Degrees.of(0.0);

    public Arm(ArmIO io) {
        this.io = io;
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
    }

    @Override
    public void periodic() {
        super.periodic();

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.updateInputs(inputs);
            Logger.processInputs("Arm", inputs);

            LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

            LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setFF(kS.get(), kG.get(), kV.get(), kA.get()), kS, kG, kV, kA);

            this.io.runSetpoint(this.setpoint);
        }

        RobotState.instance().updateArmAngle(this.inputs.position);
    }

    public Command setPosition(Angle position) {
        return runOnce(() -> this.setpoint = position);
    }

}
