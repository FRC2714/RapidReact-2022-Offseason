// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  public CANSparkMax LeftShooterMotor;
  public CANSparkMax RightShooterMotor;

  private RelativeEncoder shooterEncoder;
  private SparkMaxPIDController shooterPID;

  private CANSparkMaxLowLevel.MotorType Brushless = CANSparkMaxLowLevel.MotorType.kBrushless;
  private CANSparkMax.ControlType Velocity = CANSparkMax.ControlType.kVelocity;

  private double defaultRPM = 200;
  private double midShotRPM = 500; 
  private double targetRPM = 0;
  
  /** Creates a new Shooter. */
  public Shooter() {
    LeftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, Brushless);
    RightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorPort, Brushless);

    LeftShooterMotor.setSmartCurrentLimit(60);
    RightShooterMotor.setSmartCurrentLimit(60);

    shooterEncoder = LeftShooterMotor.getEncoder();

    RightShooterMotor.follow(RightShooterMotor, true);

    LeftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    RightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    shooterPID = LeftShooterMotor.getPIDController();
    shooterPID.setFF(ShooterConstants.kShooterFF);
    shooterPID.setP(ShooterConstants.kShooterP);
    shooterPID.setI(ShooterConstants.kShooterI);
    shooterPID.setD(ShooterConstants.kShooterD);
  }

  public void setShooterPower(double power) {
    LeftShooterMotor.set(power);
  }

  public void setTargetRpm(double targetRPM) {
    this.targetRPM = targetRPM;
    shooterPID.setReference(targetRPM, Velocity);
  }

  public void setMidShot() {
    setTargetRpm(midShotRPM);
  }

  public void setDefault() {
    setTargetRpm(defaultRPM);
  }

  public double getVelocity() {
    return shooterEncoder.getVelocity();
  }

  public boolean atSetpoint() {
    return Math.abs(-targetRPM - getVelocity()) < ShooterConstants.kVelocityTolerance;
  }

  public void disable() {
    shooterPID.setReference(0, Velocity);
    LeftShooterMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
