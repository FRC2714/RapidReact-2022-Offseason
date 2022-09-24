package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public CANSparkMax intakeMotor, pivotMotor;
  public RelativeEncoder pivotEncoder;
  public SparkMaxPIDController pivotPIDController;

  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    pivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    intakeMotor.setIdleMode(IdleMode.kCoast);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.setSmartCurrentLimit(30);
    pivotMotor.setSmartCurrentLimit(40);

    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPosition(0);
    // pivotEncoder.setPositionConversionFactor(IntakeConstants.kPivotEncoderConversionFactor);

    pivotPIDController = pivotMotor.getPIDController();
    pivotPIDController.setP(IntakeConstants.kP);
    pivotPIDController.setFF(IntakeConstants.kFF);
    pivotPIDController.setD(IntakeConstants.kD);

   
  }
  public void setPower(double power){
    intakeMotor.set(power);
  }

  public void setPivotAngle(double degrees) {
    pivotPIDController.setReference(degrees, ControlType.kPosition);
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
    setPivotAngle(0);
  }

  public void deployPivot() {
    setPivotAngle(-34);
  }

  public void periodic() {
    SmartDashboard.putNumber("intake position", pivotEncoder.getPosition());
  }

  
}
