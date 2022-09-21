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
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomSwerveControllerCommand;
import frc.robot.util.PathGenerator;

public class FiveBallAuto extends SequentialCommandGroup {
	public FiveBallAuto(SwerveSubsystem swerveSubsystem) {
		CustomSwerveControllerCommand splineToFirstBall =
			PathGenerator.PathCommand(swerveSubsystem,
				List.of(
          new Pose2d(Units.feetToMeters(24.75), Units.feetToMeters(5.8), Rotation2d.fromDegrees(-90.00)), 
          new Pose2d(Units.feetToMeters(24.75), Units.feetToMeters(2.08), Rotation2d.fromDegrees(-90.00))),
				AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

        CustomSwerveControllerCommand splineToSecondBall =
        PathGenerator.PathCommand(swerveSubsystem,
          List.of(
            new Pose2d(Units.feetToMeters(26.38), Units.feetToMeters(2.08), Rotation2d.fromDegrees(-90.00)),
            new Pose2d(Units.feetToMeters(16.7), Units.feetToMeters(6.2), Rotation2d.fromDegrees(37.00))),
          AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    CustomSwerveControllerCommand splineToHumanPlayer =
			PathGenerator.PathCommand(swerveSubsystem,
				List.of(
          new Pose2d(Units.feetToMeters(17.68), Units.feetToMeters(6.74), Rotation2d.fromDegrees(37.00)), 
          new Pose2d(Units.feetToMeters(4.3), Units.feetToMeters(4.6), Rotation2d.fromDegrees(45.00))),
				AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    CustomSwerveControllerCommand splineToGoal =
        PathGenerator.PathCommand(swerveSubsystem,
          List.of(
            new Pose2d(Units.feetToMeters(4.3), Units.feetToMeters(4.6), Rotation2d.fromDegrees(45.00)),
            new Pose2d(Units.feetToMeters(16.7), Units.feetToMeters(6.2), Rotation2d.fromDegrees(-146.0))), 
          AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

        
		addCommands(
			new InstantCommand(() -> swerveSubsystem.resetOdometry(splineToFirstBall.getInitialPose())),
			splineToFirstBall,
      splineToSecondBall,
      splineToHumanPlayer,
      splineToGoal,
			new InstantCommand(() -> swerveSubsystem.stopModules())

		);
		addCommands();
	}
}