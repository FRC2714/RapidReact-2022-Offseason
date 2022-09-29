// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drivetrain.AutoAlign;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeCommand.IntakeState;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.ShooterCommand.ShooterState;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomSwerveControllerCommand;
import frc.robot.util.PathGenerator;

public class FiveBallAuto extends SequentialCommandGroup {
  public FiveBallAuto(SwerveSubsystem swerveSubsystem, Limelight limelight, Shooter shooter, Hood hood, Intake intake,
      Index index) {
    CustomSwerveControllerCommand splineToFirstBall = PathGenerator.PathCommand(swerveSubsystem,
        List.of(
            new Pose2d(Units.feetToMeters(24.75), Units.feetToMeters(5.8), Rotation2d.fromDegrees(-90.00)),

            new Pose2d(Units.feetToMeters(24.8), Units.feetToMeters(1.7), Rotation2d.fromDegrees(-90.00))),
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    CustomSwerveControllerCommand splineToSecondBall = PathGenerator.PathCommand(swerveSubsystem,
        List.of(
            new Pose2d(Units.feetToMeters(24.8), Units.feetToMeters(1.7), Rotation2d.fromDegrees(-90.00)),
            new Pose2d(Units.feetToMeters(17.68), Units.feetToMeters(6.74), Rotation2d.fromDegrees(-143.00))),
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    CustomSwerveControllerCommand splineToHumanPlayer = PathGenerator.PathCommand(swerveSubsystem,
        List.of(
            new Pose2d(Units.feetToMeters(17.68), Units.feetToMeters(6.74), Rotation2d.fromDegrees(-143.00)),
            new Pose2d(Units.feetToMeters(4.46), Units.feetToMeters(4.67), Rotation2d.fromDegrees(-135.00))),
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    CustomSwerveControllerCommand splineToWaiting = PathGenerator.PathCommand(swerveSubsystem,
        List.of(
            new Pose2d(Units.feetToMeters(5), Units.feetToMeters(4.6), Rotation2d.fromDegrees(-135.00)),
            new Pose2d(Units.feetToMeters(7.04), Units.feetToMeters(7.25), Rotation2d.fromDegrees(-135.00))),
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    CustomSwerveControllerCommand splineToGoal = PathGenerator.PathCommand(swerveSubsystem,
        List.of(
            new Pose2d(Units.feetToMeters(7.04), Units.feetToMeters(7.25), Rotation2d.fromDegrees(-135.00)),
            new Pose2d(Units.feetToMeters(14.403), Units.feetToMeters(8.47), Rotation2d.fromDegrees(-146.0))),
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    addCommands(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(splineToFirstBall.getInitialPose())),
        // move to and intake ball
        deadline(
            splineToFirstBall,
            new IntakeCommand(intake, IntakeState.INTAKE, index)),
        // shoot first two balls
        deadline(
            new ShooterCommand(shooter, ShooterState.DYNAMIC, hood, index).withTimeout(1.25),
            new AutoAlign(swerveSubsystem, limelight)),
        // move to third ball
        deadline(
            splineToSecondBall,
            new IntakeCommand(intake, IntakeState.INTAKE, index)),
        // shoot third ball
        deadline(
            new ShooterCommand(shooter, ShooterState.DYNAMIC, hood, index).withTimeout(1.5),
            new AutoAlign(swerveSubsystem, limelight),
            new IntakeCommand(intake, IntakeState.AUTO, index)),
        // intake human player ball
        deadline(
            splineToHumanPlayer,
            new IntakeCommand(intake, IntakeState.INTAKE, index)),
        // wait for second human player ball
        deadline(
            splineToWaiting.withTimeout(.5),
            new IntakeCommand(intake, IntakeState.INTAKE, index)),
        // move to shoot
        splineToGoal,
        // shoot first two balls
        deadline(
            new ShooterCommand(shooter, ShooterState.DYNAMIC, hood, index).withTimeout(1.5),
            new AutoAlign(swerveSubsystem, limelight),
            new IntakeCommand(intake, IntakeState.AUTO, index)),
        new InstantCommand(() -> swerveSubsystem.stopModules()));
    addCommands();
  }
}