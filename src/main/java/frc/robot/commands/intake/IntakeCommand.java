package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

  private Intake intake;
  private IntakeState intakeState;
  
  public IntakeCommand(Intake intake) {
    this.intake = intake;
  }


  @Override
  public void initialize() {}

  @Override 
    public void execute (){
        switch(intakeState){
            case EXTAKE:
            intake.deployPivot();
            intake.extakeBalls();
            break;

            case INTAKE:
            intake.deployPivot();
            intake.intakeBalls();  
            break;
        }
      }

  @Override
  public void end(boolean interrupted) {
    intake.liftPivot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public enum IntakeState{
    EXTAKE,
    INTAKE,
  }
}
