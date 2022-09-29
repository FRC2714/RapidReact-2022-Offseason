// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;
import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotCommand extends CommandBase {
  public Climber climber;
  public PivotState pivotState;
  
  public PivotCommand(Climber climber, PivotState pivotState) {
    this.climber = climber;
    this.pivotState = pivotState;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch(pivotState){
      case EXTEND:
        climber.pivotUp();
        break; 
      case RETRACT:
        climber.pivotDown();
        break;
      case STOP:
        climber.disable();
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopPivot();
  }

  public enum PivotState{
    EXTEND,
    RETRACT,
    STOP
}

  @Override
  public boolean isFinished() {
    return false;
  }
}