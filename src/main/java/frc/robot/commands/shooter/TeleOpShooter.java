// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;

public class TeleOpShooter extends CommandBase {

  public Shooter shooter;
  public Hood hood;
  public ShooterState ShooterState;
  /** Creates a new TeleOpShooter. */
  public TeleOpShooter(Shooter shooter, ShooterState ShooterState, Hood hood) {
    this.shooter = shooter;
    this.ShooterState = ShooterState;
    this.hood = hood;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(ShooterState){
      case MID:
        shooter.setMidShot();
        hood.setMidShot();
        break; 
      case OFF:
        shooter.disable();
        hood.disable();
        break;
      default: 
        shooter.setDefault();
        hood.setDefault();
        break;
    }
  }

  public enum ShooterState{
    MID,
    OFF
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
