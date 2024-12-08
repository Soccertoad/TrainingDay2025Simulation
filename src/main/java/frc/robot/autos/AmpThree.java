package frc.robot.autos;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.annotations.NamedCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class AmpThree {

  @NamedCommand("AmpThree_1.1.Intake")
  public static Command _1_1_Intake(Elevator elevator, Arm arm, Wrist wrist) {
    return elevator
        .setPosition(Inches.of(30))
        .alongWith(arm.setPosition(Degrees.of(235)))
        .alongWith(wrist.setPosition(Degrees.of(180)));
  }

  @NamedCommand("AmpThree_1.2.Intake")
  public static Command _1_2_Intake() {
    return Commands.runOnce(() -> {});
    // return elevator.setPosition(Inches.of(30))
    //     .alongWith(arm.setPosition(Degrees.of(235)))
    //     .alongWith(wrist.setPosition(Degrees.of(180)));
  }
}
