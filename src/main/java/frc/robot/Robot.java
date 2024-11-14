// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.util.VirtualSubsystem;

import static edu.wpi.first.units.Units.*;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final Elevator elevator = new Elevator(new ElevatorIOSim());
  private final Arm arm = new Arm(new ArmIOSim());
  private final Wrist wrist = new Wrist(new WristIOSim());

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  public Robot() {
    m_robotContainer = new RobotContainer(elevator, arm, wrist);
  }

  @Override
  public void robotInit() {
      super.robotInit();
      Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
      Logger.addDataReceiver(new NT4Publisher());
      Logger.start();

      if (kUseLimelight) {
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (llMeasurement != null) {
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
        }
      }

  }

  @Override
  public void robotPeriodic() {
    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    wrist.setPosition(Degrees.of(0));
    elevator.setPosition(Inches.of(50)).schedule();
    arm.setPosition(Degrees.of(45)).schedule();
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
