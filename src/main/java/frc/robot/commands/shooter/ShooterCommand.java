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
  public double rpm;

  /** Creates a new TeleOpShooter. */
  public ShooterCommand(Shooter shooter, ShooterState shooterState, Hood hood, Index index) {
    this.shooter = shooter;
    this.shooterState = shooterState;
    this.hood = hood;
    this.index = index;
  }
  public ShooterCommand(Shooter shooter, ShooterState shooterState, Hood hood, Index index, double rpm) {
    this.shooter = shooter;
    this.shooterState = shooterState;
    this.hood = hood;
    this.index = index;
    this.rpm = rpm;
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
      case TUNING:
        shooter.setTuningRPM();
        hood.setDynamicPosition();
        if (shooter.atSetpoint() && hood.atSetpoint()) {
          index.moveAll(.5);
        } else {
          index.disable();
        }
        break;
      case MANUAL:
      shooter.setTargetRpm(rpm);
      hood.setDynamicPosition();
      if (shooter.atSetpoint() && hood.atSetpoint()) {
        index.moveAll(.5);
      } else {
        index.disable();
      }
      case OFF:
        shooter.disable();
        hood.disable();
        break;
    }
  }

  public enum ShooterState {
    TUNING,
    MANUAL,
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
