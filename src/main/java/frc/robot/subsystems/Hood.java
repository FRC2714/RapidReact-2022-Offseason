// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
  private CANSparkMax HoodMotor;

  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController hoodPID;

  private CANSparkMaxLowLevel.MotorType Brushless = CANSparkMaxLowLevel.MotorType.kBrushless;
  private CANSparkMax.ControlType Position = CANSparkMax.ControlType.kPosition;

  private double defaultPosition = 20;
  private double midShotPosition = 70;
  private double targetPosition = 0;

  /** Creates a new Hood. */
  public Hood() {
    HoodMotor = new CANSparkMax(HoodConstants.kHoodMotorPort, Brushless);

    HoodMotor.setSmartCurrentLimit(60);

    hoodEncoder = HoodMotor.getEncoder();

    HoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    hoodPID = HoodMotor.getPIDController();
    hoodPID.setP(HoodConstants.kHoodP);
    hoodPID.setI(HoodConstants.kHoodI);
    hoodPID.setD(HoodConstants.kHoodD);

    hoodEncoder.setPosition(0);
  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
    hoodPID.setReference(targetPosition, Position);
  }

  public void setMidShot() {
    setTargetPosition(midShotPosition);
  }

  public void setDefault() {
    setTargetPosition(defaultPosition);
  }

  public double getPosition() {
    return hoodEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(-targetPosition - getPosition()) < HoodConstants.kPositionTolerance;
  }

  public void disable() {
    hoodPID.setReference(defaultPosition, Position);
    HoodMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
