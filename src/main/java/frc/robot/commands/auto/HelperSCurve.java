package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomSwerveControllerCommand;
import frc.robot.util.PathGenerator;


public class HelperSCurve extends SequentialCommandGroup {

        public HelperSCurve(SwerveSubsystem swerveSubsystem) {

            CustomSwerveControllerCommand SCurve =
                PathGenerator.PathCommand(swerveSubsystem,
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared, false);

        

               addCommands(
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(SCurve.getInitialPose())),
                    SCurve,
                    new InstantCommand(() -> swerveSubsystem.stopModules())
                   
               );
               
        }

    }
