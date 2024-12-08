package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.util.Gains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private final Gains gains =
      Robot.isReal() ? WristConstants.TalonFXGains : WristConstants.SimGains;

  private final LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/Gains/kP", gains.kP);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Wrist/Gains/kI", gains.kI);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/Gains/kD", gains.kD);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/Gains/kS", gains.kS);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Wrist/Gains/kV", gains.kV);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Wrist/Gains/kA", gains.kA);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Wrist/Gains/kG", gains.kG);

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private Angle setpoint = Degrees.of(0.0);

  public Wrist(WristIO io) {
    this.io = io;
    this.io.setPID(kP.get(), kI.get(), kD.get());
    this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    super.periodic();

    if (edu.wpi.first.wpilibj.RobotState.isDisabled()) {
      this.io.stop();
    } else {
      this.io.updateInputs(inputs);
      Logger.processInputs("Wrist", inputs);

      LoggedTunableNumber.ifChanged(
          hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

      LoggedTunableNumber.ifChanged(
          hashCode(), () -> io.setFF(kS.get(), kG.get(), kV.get(), kA.get()), kS, kG, kV, kA);

      this.io.runSetpoint(this.setpoint);
    }

    RobotState.instance().updateWristAngle(this.inputs.position);
  }

  public Command setPosition(Angle position) {
    return runOnce(() -> this.setpoint = position);
  }
}
