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
  public void initialize() {
  }

  @Override
  public void execute() {
    switch (intakeState) {
      case EXTAKE:
        intake.deployPivot();
        intake.extakeBalls();
        index.moveAll(-.75);
        break;

      case INTAKE:
        intake.deployPivot();
        intake.intakeBalls();
        index.setRollerPower(.75);
        index.setIndexPower(.25);
        if (index.getIndexBreakbeam()) {
          index.setIndexPower(0);
          index.setRollerPower(.8);

          if (index.getRollerBreakbeam()) {
            index.setIndexPower(.075);
            index.setRollerPower(.75);
          }
        }
        break;

      case AUTO:
        intake.deployPivot();
        intake.intakeBalls();
        index.setRollerPower(1);
        break;

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

  public enum IntakeState {
    EXTAKE,
    INTAKE,
    AUTO,
  }
}
