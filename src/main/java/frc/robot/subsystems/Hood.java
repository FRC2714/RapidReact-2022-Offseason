// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants.HoodConstants;
import frc.robot.util.InterpolatingTreeMap;

public class Hood extends SubsystemBase {
  private CANSparkMax hoodMotor;
  private Limelight limelight;

  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController hoodPID;

  private double defaultPosition = 0;
  private double midShotPosition = 0;
  private double targetPosition = 0;

  public InterpolatingTreeMap hoodPosition = new InterpolatingTreeMap();

  public Hood(Limelight limelight) {

    this.limelight = limelight;

    hoodMotor = new CANSparkMax(HoodConstants.kHoodMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    hoodMotor.setSmartCurrentLimit(40);

    hoodEncoder = hoodMotor.getEncoder();

    hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    hoodPID = hoodMotor.getPIDController();
    hoodPID.setFF(HoodConstants.kHoodFF);
    hoodPID.setP(HoodConstants.kHoodP, 0);
    hoodPID.setI(HoodConstants.kHoodI, 0);
    hoodPID.setD(HoodConstants.kHoodD, 0);
    hoodPID.setSmartMotionMaxVelocity(HoodConstants.kMaxVel, 0);
    hoodPID.setSmartMotionMaxAccel(HoodConstants.kMaxAcc, 0);
    hoodPID.setSmartMotionAllowedClosedLoopError(HoodConstants.kPositionTolerance, 0);


    hoodEncoder.setPosition(0);
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, HoodConstants.kTopLimit);
    hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, HoodConstants.kBottomLimit);


    populateMap();

  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
    hoodPID.setReference(-targetPosition, ControlType.kSmartMotion, 0);
  }

  public void setMidShot() {
    setTargetPosition(midShotPosition);
  }

  public void populateMap() {
    hoodPosition.put(4.5, 1.0);
    hoodPosition.put(7.0, 4.5);
    hoodPosition.put(8.5, 10.0);
    hoodPosition.put(10.0, 14.0);
    hoodPosition.put(12.0, 18.0);
    hoodPosition.put(14.0, 25.0);
    hoodPosition.put(16.5, 35.0);
    hoodPosition.put(20.0, 37.5); 
  } // TODO: populate map

  public void setDefault() {
    setTargetPosition(defaultPosition);
  }

  public double getPosition() {
    return hoodEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(targetPosition + getPosition()) < HoodConstants.kPositionSetpointTolerance;
  }

  public double getTargetPosition() {
    return limelight.targetVisible()
        ? hoodPosition.getInterpolated(Units.metersToFeet(limelight.getDistanceToGoal())) : defaultPosition;
  }

  public void setDynamicPosition() {
    setTargetPosition(getTargetPosition());
  }

  public void disable() {
    hoodMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CURRENT HOOD POSITION", getPosition());
    SmartDashboard.putNumber("TARGET HOOD POSITION", getTargetPosition());
    SmartDashboard.putBoolean("HOOD TOLERANCE", atSetpoint());
  }
}
