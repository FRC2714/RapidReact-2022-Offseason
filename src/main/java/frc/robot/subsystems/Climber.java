package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
  private CANSparkMax LeftPivotMotor;
  private CANSparkMax RightPivotMotor;
  private CANSparkMax LeftClimbMotor;
  private CANSparkMax RightClimbMotor;

  private RelativeEncoder LeftPivotEncoder;
  private RelativeEncoder LeftClimbEncoder;
  private RelativeEncoder RightPivotEncoder;
  private RelativeEncoder RightClimbEncoder;

  public SparkMaxPIDController pivotPIDController;
  public SparkMaxPIDController climbPIDController;

  public Climber() {
    LeftPivotMotor = new CANSparkMax(ClimbConstants.kLeftPivotMotorPort, MotorType.kBrushless);
    RightPivotMotor = new CANSparkMax(ClimbConstants.kRightPivotMotorPort, MotorType.kBrushless);
    LeftClimbMotor = new CANSparkMax(ClimbConstants.kLeftClimbMotorPort, MotorType.kBrushless);
    RightClimbMotor = new CANSparkMax(ClimbConstants.kRightClimbMotorPort, MotorType.kBrushless);

    LeftPivotEncoder = LeftPivotMotor.getEncoder();
    LeftClimbEncoder = LeftClimbMotor.getEncoder();
    RightPivotEncoder = RightPivotMotor.getEncoder();
    RightClimbEncoder = RightClimbMotor.getEncoder();

    LeftClimbMotor.setSoftLimit(SoftLimitDirection.kForward, -105);

    RightPivotMotor.follow(LeftPivotMotor, true);
    RightClimbMotor.follow(LeftClimbMotor, true);

    LeftPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    RightPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    LeftClimbMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    RightClimbMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    LeftPivotEncoder.setPosition(0);
    LeftClimbEncoder.setPosition(0);
    RightPivotEncoder.setPosition(0);
    RightClimbEncoder.setPosition(0);

    // LeftClimbMotor.setSoftLimit(SoftLimitDirection.kForward,
    // ClimbConstants.kMaxHeight);

    // Set Current Limits
    LeftClimbMotor.setSmartCurrentLimit(50);
    RightClimbMotor.setSmartCurrentLimit(50);

    // Pivot PID
    pivotPIDController = LeftPivotMotor.getPIDController();
    pivotPIDController.setFF(ClimbConstants.kPivotFF);
    pivotPIDController.setP(ClimbConstants.kPivotP);
    pivotPIDController.setP(ClimbConstants.kPivotD);

    // Climb PID
    climbPIDController = LeftClimbMotor.getPIDController();
    climbPIDController.setFF(ClimbConstants.kClimbFF);
    climbPIDController.setP(ClimbConstants.kClimbP);

  }

  // Pivot
  public void pivotDown() {
    LeftPivotMotor.set(ClimbConstants.kPivotSpeed);
  }

  public void pivotUp() {
    LeftPivotMotor.set(-ClimbConstants.kPivotSpeed);
  }

  public void setPivot(double position) {
    pivotPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  // Climber
  public void climberUp() {
    LeftClimbMotor.set(ClimbConstants.kClimbSpeed);
  }

  public void climberDown() {
    LeftClimbMotor.set(-ClimbConstants.kClimbSpeed);
  }

  // Stop
  public void stopPivot() {
    LeftPivotMotor.set(0);
  }

  public void stopClimber() {
    LeftClimbMotor.set(0);
  }

  public void disable() {
    LeftPivotMotor.set(0);
    LeftClimbMotor.set(0);
  }

  // Get Position
  public void getPivotPosition() {
    LeftPivotEncoder.getPosition();
  }

  public void getClimbPosition() {
    LeftClimbEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Pivot Encoder", LeftPivotEncoder.getPosition());
    SmartDashboard.putNumber("Right Pivot Encoder", RightPivotEncoder.getPosition());

    SmartDashboard.putNumber("Left Climb Encoder", LeftClimbEncoder.getPosition());
    SmartDashboard.putNumber("Right Climb Encoder", RightClimbEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}