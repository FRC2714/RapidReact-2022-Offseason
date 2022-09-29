// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index.IndexState;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Index;

public class ShooterCommand extends CommandBase {

  public Shooter shooter;
  public Hood hood;
  public ShooterState shooterState;
  public Index index;

  /** Creates a new TeleOpShooter. */
  public ShooterCommand(Shooter shooter, ShooterState shooterState, Hood hood, Index index) {
    this.shooter = shooter;
    this.shooterState = shooterState;
    this.hood = hood;
    this.index = index;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shooterState) {
      case DYNAMIC:
        shooter.setDynamicRpm();
        hood.setDynamicPosition();
        if (shooter.atSetpoint() && hood.atSetpoint()) {
          index.moveAll(.5);
        } else {
          index.disable();
        }
        break;
      case LOW:
        shooter.setTargetRpm(1000);
        hood.setTargetPosition(0);
        if (shooter.atSetpoint() && hood.atSetpoint()) {
          index.moveAll(.5);
        } else {
          index.disable();
        }
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

  public enum ShooterState {
    LOW,
    OFF,
    DYNAMIC,
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disable();
    index.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
