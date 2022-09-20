package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index.IndexState;

public class IntakeCommand extends CommandBase {

  private Intake intake;
  private IntakeState intakeState;
  private Index index;
  
  public IntakeCommand(Intake intake, IntakeState intakeState, Index index) {
    this.intake = intake;
    this.intakeState = intakeState;
    this.index = index;
  }


  @Override
  public void initialize() {}

  @Override 
    public void execute (){
        switch(intakeState){
            case EXTAKE:
            intake.deployPivot();
            intake.extakeBalls();
            index.moveAll(-.5);
            break;

            case INTAKE:
            intake.deployPivot();
            intake.intakeBalls();
            index.moveAll(.25);
            if (index.getIndexBreakbeam()) {
              index.setIndexPower(0);
              index.setRollerPower(.25);

              if (index.getRollerBreakbeam()) {
                  index.moveAll(.25);
              }
            break;
            }
          }
      }

  @Override
  public void end(boolean interrupted) {
    intake.liftPivot();
    intake.setPower(0);
    index.moveAll(0);
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
