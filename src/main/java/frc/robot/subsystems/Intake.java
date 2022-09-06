package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public CANSparkMax intakeMotor, pivotMotor;
  public RelativeEncoder pivotEncoder;
  public SparkMaxPIDController pivotPIDController;

  public double targetDegrees(double angle){
    angle /= 360;
    angle *= IntakeConstants.kPivotGearRatio;
    return angle;
  }

  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    pivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    intakeMotor.setSmartCurrentLimit(30);
    pivotMotor.setSmartCurrentLimit(30);

    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPosition(0);

    pivotPIDController = pivotMotor.getPIDController();
    pivotPIDController.setP(IntakeConstants.kP);
    pivotPIDController.setFF(IntakeConstants.kFF);
    pivotPIDController.setD(IntakeConstants.kD);

   
  }
  public void setPower(double power){
    intakeMotor.set(power);
  }

  public void setPivotAngle(double degrees) {
    pivotPIDController.setReference(targetDegrees(degrees), ControlType.kPosition);
  }

  public void intakeBalls() {
    intakeMotor.set(IntakeConstants.kIntakePower);
  }

  public void extakeBalls() {
    intakeMotor.set(-IntakeConstants.kIntakePower);
  }

  public void zeroPivot() {
    pivotEncoder.setPosition(0);
  }

  public void liftPivot() {
    setPivotAngle(5);
  }

  public void deployPivot() {
    setPivotAngle(20);
  }

  
}
