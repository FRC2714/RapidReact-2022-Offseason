// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlign extends ProfiledPIDCommand {
  private SwerveSubsystem swerveSubsystem;
  private Limelight limelight;
  /** Creates a new AutoAlign. */
  public AutoAlign(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            1,
            0,
            0.01,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared)),
        // This should return the measurement
        limelight::getXRadianOffset,
        // This should return the goal (can also be a constant)
        0,
        // This uses the output
        (output, setpoint) -> swerveSubsystem.rawDrive(new ChassisSpeeds(0,0,output * 4))
          // Use the output (and setpoint, if desired) here
        );
        addRequirements(swerveSubsystem);
        this.limelight = limelight;
        this.swerveSubsystem = swerveSubsystem;
        getController().setTolerance(.1, .2);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted){
      swerveSubsystem.stopModules();
  }
}
