// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;
import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveClimber extends CommandBase {
  public Climber climber;
  public ClimberState ClimberState;
  
  /** Creates a new MoveClimb. */
  public MoveClimber(Climber climber, ClimberState ClimberState) {
    this.climber = climber;
    this.ClimberState = ClimberState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(ClimberState){
      case EXTEND:
        climber.climberUp();
        break; 
      case RETRACT:
        climber.climberDown();
        break;
      case STOP:
        climber.disable();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.disable();
  }

  public enum ClimberState{
    EXTEND,
    RETRACT,
    STOP
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}