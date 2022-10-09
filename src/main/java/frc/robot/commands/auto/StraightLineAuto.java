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

public class StraightLineAuto extends SequentialCommandGroup {
	private double initialX = 19.40;
    private double distance = 12.0;
    
    public StraightLineAuto(SwerveSubsystem swerveSubsystem, Limelight limelight, Shooter shooter, Hood hood, Intake intake, Index index) {
		CustomSwerveControllerCommand moveBack =
			PathGenerator.PathCommand(swerveSubsystem,
				List.of(
                    new Pose2d(Units.feetToMeters(10), Units.feetToMeters(12.56), Rotation2d.fromDegrees(180.00)), 
                    new Pose2d(Units.feetToMeters(0), Units.feetToMeters(12.56), Rotation2d.fromDegrees(180.00))),
				AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

		addCommands(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(moveBack.getInitialPose())),
            //move back with intake on
            deadline(
                moveBack, 
                new IntakeCommand(intake, IntakeState.INTAKE, index)
            ),
            //shoot preload ball
            deadline(
                new ShooterCommand(shooter, ShooterState.DYNAMIC, hood, index).withTimeout(1.5), 
                new AutoAlign(swerveSubsystem, limelight)
            ),
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
		addCommands();
	}
}