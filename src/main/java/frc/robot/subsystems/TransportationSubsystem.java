// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TransportationSubsystem extends SubsystemBase {
  WPI_TalonSRX intakeLowerMotor = new WPI_TalonSRX(Constants.TransportationConstants.INTAKE_LOWER_MOTOR_PORT);
  WPI_VictorSPX intakeHigherMotor = new WPI_VictorSPX(Constants.TransportationConstants.INTAKE_HIGHER_MOTOR_PORT);

  /** Creates a new IntakeSubsystem. */
  public TransportationSubsystem() {
    intakeHigherMotor.setInverted(true);
    intakeLowerMotor.setInverted(true);
    RobotContainer.transportationMotor.setInverted(true);
  }

  public void setSpeed(double intakeLowerMotorSpeed, double intakeHigherMotorSpeed, double transportationMotorSpeed) {
    intakeHigherMotor.set(intakeHigherMotorSpeed);
    intakeLowerMotor.set(intakeLowerMotorSpeed);
    RobotContainer.transportationMotor.set(transportationMotorSpeed);

  }

  public void stopMotors() {
    intakeHigherMotor.set(0);
    intakeLowerMotor.set(0);
    RobotContainer.transportationMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
