package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
  private CANSparkMax LeftPivotMotor;
  private CANSparkMax RightPivotMotor;
  private CANSparkMax LeftClimbMotor;
  private CANSparkMax RightClimbMotor;

  private RelativeEncoder LeftPivotEncoder;
  private RelativeEncoder LeftClimbEncoder;

  private CANSparkMaxLowLevel.MotorType Brushless = CANSparkMaxLowLevel.MotorType.kBrushless;

  private int MinPivot = 0;
  private int MaxPivot = 100;//TEST
  private double pivotDownSpeed = 0.5;
  private double pivotUpSpeed = -0.5;

  private int MinHeight = 0;
  private int MaxHeight = 100;//TEST
  private double climbUpSpeed = 0.5;
  private double climbDownSpeed = -0.5;

  public Climber() {
    LeftPivotMotor = new CANSparkMax(ClimbConstants.kLeftPivotMotorPort, Brushless);
    RightPivotMotor = new CANSparkMax(ClimbConstants.kRightPivotMotorPort, Brushless);
    LeftClimbMotor = new CANSparkMax(ClimbConstants.kLeftPivotMotorPort, Brushless);
    RightClimbMotor = new CANSparkMax(ClimbConstants.kRightPivotMotorPort, Brushless);

    LeftPivotEncoder = LeftPivotMotor.getEncoder();
    LeftClimbEncoder = LeftClimbMotor.getEncoder();
    
    RightPivotMotor.follow(LeftPivotMotor, true);
    RightClimbMotor.follow(LeftClimbMotor, true);

    LeftPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    RightPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    LeftClimbMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    RightClimbMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    LeftPivotEncoder.setPosition(0);
    LeftClimbEncoder.setPosition(0);
  }

  //Pivot
  public void pivotDown() {
    if(LeftPivotEncoder.getPosition() < MaxPivot) {
      LeftPivotMotor.set(pivotDownSpeed);
    }
    stopPivot();
  }

  public void pivotUp() {
    if(LeftPivotEncoder.getPosition() > MinPivot) {
      LeftPivotMotor.set(pivotUpSpeed);
    }
    stopPivot();
  }

  //Climber
  public void climberUp() {
    if(LeftClimbEncoder.getPosition() < MaxHeight) {
      LeftClimbMotor.set(climbUpSpeed);
    }
    stopPivot();
  }

  public void climberDown() {
    if(LeftClimbEncoder.getPosition() > MinHeight) {
      LeftClimbMotor.set(climbDownSpeed);
    }
    stopPivot();
  }

  //Stop
  public void stopPivot() {
    LeftPivotMotor.set(0);
  }

  public void stopClimber() {
    LeftClimbMotor.set(0);
  }

  public void stopAll() {
    stopPivot();
    stopClimber();
  }

  //Get Position
  public void getPivotPosition() {
    LeftPivotEncoder.getPosition();
  }

  public void getClimbPosition() {
    LeftClimbEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Encoder", LeftPivotEncoder.getPosition());
    SmartDashboard.putNumber("Climb Encoder", LeftClimbEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}