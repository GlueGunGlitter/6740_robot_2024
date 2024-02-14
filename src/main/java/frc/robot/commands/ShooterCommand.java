// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  int _state = 0;

  /** Creates a new ShooterCommand. */
  public ShooterCommand() {
    addRequirements(RobotContainer.m_ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public void spinUp(double nonStaticMotorSpeed, double staticMotorSpeed) {
    RobotContainer.m_ShooterSubsystem.shootUp(nonStaticMotorSpeed, staticMotorSpeed, 1);
  }

  public void spinDown(double nonStaticMotorSpeed, double staticMotorSpeed) {
    RobotContainer.m_ShooterSubsystem.shootDown(nonStaticMotorSpeed - 0.2, staticMotorSpeed - 0.2, 1);
  }

  public void stopShooter() {
    RobotContainer.m_ShooterSubsystem.stopMotors();

  }

  public void setState(int state) {
    this._state = state;
  }

  public int getState() {
    return this._state;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double staticMotorSpeed = Robot.staticShooterMotorSpeed.getDouble(0);
    double nonStaticMotorSpeed = Robot.nonStaticShooterMotorSpeed.getDouble(0);

    if (RobotContainer.xboxController.getAButton() || RobotContainer.xboxController.getAButton()) {
      RobotContainer.transportationShooter = !RobotContainer.transportationShooter;
    }
    if (DriverStation.isTeleop()) {
      if (RobotContainer.xboxController.getAButton()) {
        spinUp(nonStaticMotorSpeed, staticMotorSpeed);
      } else if (RobotContainer.xboxController.getBButton()) {
        spinDown(nonStaticMotorSpeed - 0.2, staticMotorSpeed - 0.2);
      } else {
        stopShooter();
      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
