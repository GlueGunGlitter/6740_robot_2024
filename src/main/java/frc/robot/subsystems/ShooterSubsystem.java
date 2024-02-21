// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkFlex nonStaticMotor = new CANSparkFlex(Constants.ShooterConstants.NON_STATIC_MOTOR_PORT,
      MotorType.kBrushless);
  CANSparkFlex staticMotor = new CANSparkFlex(Constants.ShooterConstants.STATIC_MOTOR_PORT,
      MotorType.kBrushless);
  PIDController staticMotorPID = new PIDController(5, 0, 0);
  PIDController nonStaticMotorPID = new PIDController(5, 0, 0);

  private int setPoint = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    nonStaticMotor.setInverted(true);
  }

  public void shootUp(double nonStaticMotorSpeed, double staticMotorSpeed) {
    nonStaticMotorPID.setSetpoint(-nonStaticMotorSpeed);
    staticMotorPID.setSetpoint(staticMotorSpeed);;
  }

  public void shootDown(double nonStaticMotorSpeed, double staticMotorSpeed) {
    nonStaticMotorPID.setSetpoint(nonStaticMotorSpeed);
    staticMotorPID.setSetpoint(staticMotorSpeed);
  }

  public void stopMotors() {
    nonStaticMotorPID.setSetpoint(0);
    staticMotorPID.setSetpoint(0);
  }

  private double getVelocityOfNonStaticMotor() {
    return nonStaticMotor.getEncoder().getVelocity();
  }

  private double getVelocityOfStaticMotor() {
    return staticMotor.getEncoder().getVelocity();
  }

  private double calculateSpeedToStaticMotor() {
    return staticMotorPID.calculate(getVelocityOfStaticMotor())
        / Constants.ShooterConstants.MAX_SPEED_OF_STATIC_MOTOR;
  }

  private double calculateSpeedToNonStaticMotor() {
    return nonStaticMotorPID.calculate(getVelocityOfNonStaticMotor())
        / Constants.ShooterConstants.MAX_SPEED_OF_NON_STATIC_MOTOR;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    nonStaticMotor.set(calculateSpeedToNonStaticMotor());
    staticMotor.set(calculateSpeedToStaticMotor());
  }
}
