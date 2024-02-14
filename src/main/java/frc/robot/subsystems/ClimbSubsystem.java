// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  WPI_VictorSPX climbRightMotor = new WPI_VictorSPX(Constants.ClimbConstants.CLIMB_RIGHT_MOTOR_PORT);
  WPI_TalonSRX climbLeftMotor = new WPI_TalonSRX(Constants.ClimbConstants.CLIMB_LEFT_MOTOR_PORT);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
  }

  public void setSpeedLeft(double climbLeftMotorSpeed) {
    climbLeftMotor.set(climbLeftMotorSpeed);
  }

  public void setSpeedRight(double climbRightMotorSpeed) {
    climbRightMotor.set(climbRightMotorSpeed);
  }

  public void stopMotor() {
    climbLeftMotor.set(0);
    climbRightMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
