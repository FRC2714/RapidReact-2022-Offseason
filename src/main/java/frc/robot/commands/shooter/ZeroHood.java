package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class ZeroHood extends CommandBase {

  private Hood hood;
  private boolean finished = false;

  public ZeroHood(Hood hood) {
    this.hood = hood;
  }



  @Override
  public void initialize() {
    finished = false;
    hood.set(.1);
  }


  @Override
  public void execute() {
    if (hood.getOutputCurrent() > 20) {
      hood.set(0);
      hood.setZero();
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return finished;
  }
}
