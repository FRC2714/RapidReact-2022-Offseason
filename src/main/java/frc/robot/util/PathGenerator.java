package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;


import java.util.List;

public class PathGenerator {


    private static TrajectoryConfig getConfig(double maxVel, double maxAcc) {

        return new TrajectoryConfig(maxVel, maxAcc)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

    }

    public static CustomSwerveControllerCommand PathCommand(SwerveSubsystem swerveSubsystem,
                                                         Pose2d startPose,
                                                         List<Translation2d> internalPoints,
                                                         Pose2d endPose,
                                                         double velocity, double acceleration) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,

                internalPoints,

                endPose,

                getConfig(velocity, acceleration)
        );

        return getPathCommand(swerveSubsystem, trajectory);
    }


    public static CustomSwerveControllerCommand PathCommand(SwerveSubsystem swerveSubsystem,
                                                         List<Pose2d> waypoints,
                                                         double velocity, double acceleration) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                waypoints,
                getConfig(velocity, acceleration)
        );

        return getPathCommand(swerveSubsystem, trajectory);
    }


    private static CustomSwerveControllerCommand getPathCommand(SwerveSubsystem swerveSubsystem, Trajectory trajectory) {
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(Math.PI, -Math.PI);

        return new CustomSwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem
        );
    }

}