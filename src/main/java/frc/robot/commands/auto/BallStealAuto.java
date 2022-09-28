// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrain.AutoAlign;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeCommand.IntakeState;
import frc.robot.commands.shooter.TeleOpShooter;
import frc.robot.commands.shooter.TeleOpShooter.ShooterState;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomSwerveControllerCommand;
import frc.robot.util.PathGenerator;

public class BallStealAuto extends SequentialCommandGroup {
	public BallStealAuto(SwerveSubsystem swerveSubsystem, Limelight limelight, Shooter shooter, Hood hood, Intake intake, Index index) {
		CustomSwerveControllerCommand splineToTeamBall =
			PathGenerator.PathCommand(swerveSubsystem,
				List.of(
          new Pose2d(Units.feetToMeters(19.73), Units.feetToMeters(16.86), Rotation2d.fromDegrees(135)), 
          new Pose2d(Units.feetToMeters(16.59), Units.feetToMeters(19.78), Rotation2d.fromDegrees(130.00))),
				AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    CustomSwerveControllerCommand splineToOpposingHangarBall =
      PathGenerator.PathCommand(swerveSubsystem,
        List.of(
          new Pose2d(Units.feetToMeters(16.59), Units.feetToMeters(19.78), Rotation2d.fromDegrees(130.00)),
          new Pose2d(Units.feetToMeters(19.55), Units.feetToMeters(23.39), Rotation2d.fromDegrees(-1.00))),
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    CustomSwerveControllerCommand splineToOpposingHubBall =
      PathGenerator.PathCommand(swerveSubsystem,
        List.of(
          new Pose2d(Units.feetToMeters(19.55), Units.feetToMeters(23.39), Rotation2d.fromDegrees(-1.00)),
          new Pose2d(Units.feetToMeters(14.40), Units.feetToMeters(11.34), Rotation2d.fromDegrees(-73.00))),
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    CustomSwerveControllerCommand splineToHub =
      PathGenerator.PathCommand(swerveSubsystem,
        List.of(
          new Pose2d(Units.feetToMeters(14.40), Units.feetToMeters(11.34), Rotation2d.fromDegrees(-73.00)),
          new Pose2d(Units.feetToMeters(18.51), Units.feetToMeters(12.07), Rotation2d.fromDegrees(19.0))),
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

		addCommands(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(splineToTeamBall.getInitialPose())),
      //move to and intake team ball
      deadline(
        splineToTeamBall, 
        new IntakeCommand(intake, IntakeState.INTAKE, index)
      ),
      //shoot first two balls
      deadline(
        new TeleOpShooter(shooter, ShooterState.DYNAMIC, hood, index).withTimeout(2.5), 
        new IntakeCommand(intake, IntakeState.AUTO, index),
        new AutoAlign(swerveSubsystem, limelight)
      ),
      //move to and intake opposing hanger ball
      deadline(
        splineToOpposingHangarBall, 
        new IntakeCommand(intake, IntakeState.INTAKE, index)
      ),
      //move to and intake opposing hub ball
      deadline(
        splineToOpposingHubBall, 
        new IntakeCommand(intake, IntakeState.INTAKE, index)
      ),
      //move to hub
      splineToHub,
      //extake both balls
      new IntakeCommand(intake, IntakeState.EXTAKE, index).withTimeout(3.0),

			new InstantCommand(() -> swerveSubsystem.stopModules())
		);
		addCommands();
	}
}