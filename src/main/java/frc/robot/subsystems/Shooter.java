// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.util.InterpolatingTreeMap;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax LeftShooterMotor;
  private CANSparkMax RightShooterMotor;

  private RelativeEncoder shooterEncoder;
  private SparkMaxPIDController shooterPID;

  private InterpolatingTreeMap shooterVelocity = new InterpolatingTreeMap();
  private Limelight limelight;

  private double defaultRPM = 200;
  private double targetRPM = 0;
  
  public Shooter(Limelight limelight) {

    this.limelight = limelight;

    LeftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    RightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    LeftShooterMotor.setIdleMode(IdleMode.kCoast);
    RightShooterMotor.setIdleMode(IdleMode.kCoast);

    LeftShooterMotor.setSmartCurrentLimit(40);
    RightShooterMotor.setSmartCurrentLimit(40);

    shooterEncoder = LeftShooterMotor.getEncoder();

    RightShooterMotor.follow(LeftShooterMotor, true);

    LeftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    RightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    populateVelocityMap();

    shooterPID = LeftShooterMotor.getPIDController();
    shooterPID.setFF(ShooterConstants.kShooterFF);
    shooterPID.setP(ShooterConstants.kShooterP);
    shooterPID.setI(ShooterConstants.kShooterI);
    shooterPID.setD(ShooterConstants.kShooterD);

  }

  private void populateVelocityMap() {
        shooterVelocity.put(3.0, 200.0);
        shooterVelocity.put(6.0, 450.0);
        shooterVelocity.put(9.0, 600.0);
        shooterVelocity.put(15.0, 750.0); // TODO: populate tree map
  }

  public void setShooterPower(double power) {
    LeftShooterMotor.set(-power);
  }

  public double getTargetRpm() {
    return limelight.targetVisible()
        ? shooterVelocity.getInterpolated(Units.metersToFeet(limelight.getDistanceToGoal())) : defaultRPM;
  }

  public void setTargetRpm(double targetRPM) {
    shooterPID.setReference(-targetRPM, ControlType.kVelocity);
  }

  public void setDynamicRpm() {
    setTargetRpm(getTargetRpm());
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
   setShooterPower(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CURRENT SHOOTER RPM", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("RPM SETPOINT", getTargetRpm());
    SmartDashboard.putBoolean("SHOOTER TOLERANCE", atSetpoint());
  }
}
