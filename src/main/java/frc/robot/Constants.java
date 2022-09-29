// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.14;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kTurnP = 0.25;
        public static final double kDriveP = 0.7;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(27);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(27);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Drivetrain Motor Ports
        public static final int kFrontLeftDriveMotorPort = 6;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 2;

        public static final int kFrontLeftTurningMotorPort = 5;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 1;

        // Relative Encoders
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        // Absolute Encoders
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(46.406);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(70.928);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(252.070);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(355.166);

        // Physical
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.8;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        // Speed and Accel
        public static final double kTeleDriveMaxSpeedMultiplier = 1;
        public static final double kTeleDriveMaxAnglularSpeedMultiplier = 0.4;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond
                * kTeleDriveMaxSpeedMultiplier;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                * kTeleDriveMaxAnglularSpeedMultiplier;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 10;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 1.9;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        public static final double kPXController = .7;
        public static final double kPYController = .6;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorPort = 15;
        public static final int kIntakePivotMotorPort = 14;

        public static final double kIntakePower = 1;

        public static final double kPivotGearRatio = 1 / 225;
        public static final double kPivotEncoderConversionFactor = kPivotGearRatio * 360;
        public static final double kP = 0.11;
        public static final double kFF = 0.0039;
        public static final double kD = 0.1;

    }

    public static final class IndexConstants {

        public static final int kIndexMotorPort = 12;
        public static final int kRollerMotorPort = 13;
        public static final int kIndexBeamChannel = 0;
        public static final int kRollerBeamChannel = 1;

    }

    public static final class ShooterConstants {
        public static final int kLeftShooterMotorPort = 9;
        public static final int kRightShooterMotorPort = 10;

        public static final double kVelocityTolerance = 80;
        public static final double kShooterFF = 0.00018;
        public static final double kShooterP = 0.000;
        public static final double kShooterI = 0;
        public static final double kShooterD = 0;

    }

    public static final class FieldConstants {
        public static final double kGoalHeight = Units.inchesToMeters(98.25);
    }

    public static final class CameraConstants {
        public static final double kMountingAngle = 33;
        public static double kCameraHeight = Units.inchesToMeters(31);
    }

    public static final class HoodConstants {
        public static final int kHoodMotorPort = 11;

        public static final double kRotationtoDegrees = 0;
        public static final float kTopLimit = -40;
        public static final float kBottomLimit = 0;
        public static final double kPositionTolerance = .25;
        public static final double kPositionSetpointTolerance = 2;

        public static final double kHoodFF = 0.00028;
        public static final double kHoodP = 0.00005;
        public static final double kHoodI = 0;
        public static final double kHoodD = 0;
        public static final double kMaxAcc = 5000;
        public static final double kMaxVel = 5000;

    }

    public static final class ClimbConstants {
        // Motor ports
        public static final int kLeftPivotMotorPort = 16;
        public static final int kRightPivotMotorPort = 18;
        public static final int kLeftClimbMotorPort = 19;
        public static final int kRightClimbMotorPort = 17;

        // Climb Speeds
        public static final double kPivotSpeed = 0.015;
        public static final double kClimbSpeed = 1;

        // Climb Limits
        public static final float kMaxHeight = 100;

        public static final int kPivotMotorGearRatio = 1 / 100;
        public static final double kPivotEncoderConversionFactor = kPivotMotorGearRatio * 360;
        public static final int kClimbMotorGearRatio = 1 / 16;

        // Pivot PID
        public static final double kPivotFF = 0.05;
        public static final double kPivotP = 0.04;
        public static final double kPivotD = 0;

        // Climb PID
        public static final double kClimbFF = 0;
        public static final double kClimbP = 0;
    }

}
