// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.HoodConstants;
import frc.robot.util.InterpolatingTreeMap;

public class Hood extends SubsystemBase {
  private CANSparkMax hoodMotor;
  private Limelight limelight;

  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController hoodPID;

  private double defaultPosition = 20;
  private double midShotPosition = 70;
  private double targetPosition = 0;

  public InterpolatingTreeMap hoodPosition = new InterpolatingTreeMap();

  /** Creates a new Hood. */
  public Hood() {
    hoodMotor = new CANSparkMax(HoodConstants.kHoodMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    hoodMotor.setSmartCurrentLimit(60);

    hoodEncoder = hoodMotor.getEncoder();

    hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    hoodPID = hoodMotor.getPIDController();
    hoodPID.setP(HoodConstants.kHoodP);
    hoodPID.setI(HoodConstants.kHoodI);
    hoodPID.setD(HoodConstants.kHoodD);

    hoodEncoder.setPosition(0);
    // hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 0)
    // hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, 0)

    populateMap();

  }

  public void setTargetPosition(double targetPosition) {
    hoodPID.setReference(targetPosition, ControlType.kPosition);
  }

  public void setMidShot() {
    setTargetPosition(midShotPosition);
  }

  public void populateMap() {
    hoodPosition.put(3.0, 200.0);
    hoodPosition.put(6.0, 450.0);
    hoodPosition.put(9.0, 600.0);
    hoodPosition.put(15.0, 750.0); 
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

  public double getHoodPosition() {
    return limelight.targetVisible()
        ? hoodPosition.getInterpolated(Units.metersToFeet(limelight.getDistanceToGoal())) : defaultPosition;
  }

  public void setDynamicPosition() {
    setTargetPosition(getHoodPosition());
  }

  public void disable() {
    hoodPID.setReference(defaultPosition, ControlType.kPosition);
    hoodMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
