// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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
        public static final double kTurnP = 0.3;
        public static final double kDriveP = 0.7;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(27);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(27);


        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
        
        //Drivetrain Motor Ports
        public static final int kFrontLeftDriveMotorPort = 6;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 2;

        public static final int kFrontLeftTurningMotorPort = 5;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 1;

        //Relative Encoders
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        //Absolute Encoders
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double degToRad = Math.PI/180;
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(46.406); 
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(70.928); 
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(252.070);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(355.166); 

        //Physical
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.8;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        //Speed and Accel
        public static final double kTeleDriveMaxSpeedMultiplier = 0.5;
        public static final double kTeleDriveMaxAnglularSpeedMultiplier = 0.5;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * kTeleDriveMaxSpeedMultiplier;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * kTeleDriveMaxAnglularSpeedMultiplier;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 10;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
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

        public static final double kDeadband = 0.05;
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
}