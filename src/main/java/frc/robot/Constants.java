// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;
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
  public static final boolean tuningMode = true;

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class GlobalSensors {
    public static final int kIntakeForwardLaserCanID = 52;
    public static final int kIntakeRearLaserCanID = 53;
  }

  public static final class PathFollowingConstraints {
    public static final PathConstraints kStagePathConstraints =
        new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(540));
  }

  public static final class FrontWristConstants {
    public static final int kFrontWristMotorID = 34;

    public static final double kMaxVelocity = 1; // rad/sec
    public static final double kMaxAcceleration = 1; // rad/sec/sec
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int kSmartCurrentLimit = 60; // A

    public static final double kMinRads = Units.degreesToRadians(90);
    public static final double kMaxRads = Units.degreesToRadians(315);
    public static final double kStowedRads = Units.degreesToRadians(300);
  }

  public static final class LevetatorConstants {
    public static final int kLevetatorMotorID = 39;
    public static final int kLevetatorLaserCanID = 55;

    public static final double kLevetatorOffset = Units.inchesToMeters(.875);

    public static final double kMaxVelocity = 1.75; // m/s
    public static final double kMaxAcceleration = 0.75; // m/s^2
    public static final double kP = 1.3;
    public static final double kI = 0.0;
    public static final double kD = 0.7;
    public static final double kS = 1.1;
    public static final double kG = 1.2;
    public static final double kV = 1.3;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAngularAcceleration =
        kMaxAngularSpeed * 1.5; // FIND Actual number

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kModuleInsetMeters = Units.inchesToMeters(1.75);
    public static final double kRobotChassisLengthMeters = Units.inchesToMeters(27);
    public static final double kRobotChassisWidthMeters = Units.inchesToMeters(27);
    public static final double kTrackWidthMeters =
        kRobotChassisLengthMeters - (2 * kModuleInsetMeters);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBaseMeters =
        kRobotChassisWidthMeters - (2 * kModuleInsetMeters);
    // Distance between front and back wheels on robot
    public static final double kRobotDriveRadiusMeters =
        Math.abs(Math.hypot(kTrackWidthMeters / 2, kWheelBaseMeters / 2));
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2),
            new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
            new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2),
            new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // Module Configuration (Ensure Driving is (Mod# * 10 + 1) and Turning is (Mod# * 10 + 2))

    public static final int kFrontLeftModuleNum = 1;
    public static final int kFrontRightModuleNum = 2;
    public static final int kRearLeftModuleNum = 3;
    public static final int kRearRightModuleNum = 4;

    public static final int kDrivingIdOffset = 1;
    public static final int kTurningIdOffset = 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = kFrontLeftModuleNum * 10 + kDrivingIdOffset;
    public static final int kFrontRightDrivingCanId = kFrontRightModuleNum * 10 + kDrivingIdOffset;
    public static final int kRearLeftDrivingCanId = kRearLeftModuleNum * 10 + kDrivingIdOffset;
    public static final int kRearRightDrivingCanId = kRearRightModuleNum * 10 + kDrivingIdOffset;

    public static final int kFrontLeftTurningCanId = kFrontLeftModuleNum * 10 + kTurningIdOffset;
    public static final int kFrontRightTurningCanId = kFrontRightModuleNum * 10 + kTurningIdOffset;
    public static final int kRearLeftTurningCanId = kRearLeftModuleNum * 10 + kTurningIdOffset;
    public static final int kRearRightTurningCanId = kRearRightModuleNum * 10 + kTurningIdOffset;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class LeftClimberConstants {
    public static final int kMotorID = 30;
    public static final int kSensorID = 50;
  }

  public static final class RightClimberConstants {
    public static final int kMotorID = 31;
    public static final int kSensorID = 51;
  }

  public static final class VisionConstants {
    public static final String kFrontLimelightName = "Front_Limelight";
    public static final String kRearLimelightName = "Rear_Limelight";
  }

  public static final class ClimberConstants {
    public static final double kMaxVelocity = 0.25; // mm/s
    public static final double kMaxAcceleration = 0.25; // mm/s^2
    public static final double kP = 1.3;
    public static final double kI = 0.0;
    public static final double kD = 0.7;
    public static final double kS = 1.1;
    public static final double kG = 1.2;
    public static final double kV = 1.3;

    public static final double extensionSpeed = .1;
    public static final double retractionSpeed = .5;

    public static final double kSensorOffset = Units.inchesToMeters(2.5);
  }

  public static final class ShooterConstants {
    public static final int kLeftShooterCanID = 34;
    public static final int kRightShooterCanID = 35;
    public static final boolean kLeftMotorInverted = true;
    public static final boolean kRightMotorInverted = !kLeftMotorInverted;
    public static final int kCurrentLimit = 80; // Amps

    public static final double kP_right = 6e-5;
    public static final double kI_right = 0;
    public static final double kD_right = 0;
    public static final double kFF_right = 0.000015;
    public static final double kIz_right = 0;

    public static final double kP_left = 6e-5;
    public static final double kI_left = 0;
    public static final double kD_left = 0;
    public static final double kFF_left = 0.000015;
    public static final double kIz_left = 0;

    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double kMaxRPM = 6500;
  }

  public static final class IntakeConstants {
    public static final int kMotorID = 32;
    public static final int kCurrentLimit = 80;
  }

  public static final class LevetatorPivotConstants {
    public static final double kGearRatio = 111.1 / 1.0;
    public static final int kLeadMotorId = 37;
    public static final int kFollowerMotorId = 38;
    public static final int kMotorCurrentLimit = 85;
    public static final boolean kLeadMotorInverted = false;
    public static final boolean kEncoderInverted = true;
    public static final int kPivotUpPIDSlot = 0;
    public static final int kPivotDownPIDSlot = 1;
    public static final double kPivotMinOutput = -1;
    public static final double kPivotMaxOutput = 1;
    public static final double kPivotEncoderPositionFactor = Units.degreesToRadians(360); // rads
    public static final double kPivotEncoderVelocityFactor =
        Units.degreesToRadians(360.0) / 60.0; // rads per second
    public static final double kPivotOffset = Units.degreesToRadians(90);
  }

  public static final class PivotConstants {
    public static final int kLeadMotorPort = 37;
    public static final int kFollowerMotorPort = 38;

    public static final int kMotorCurrentLimit = 85;

    public static final boolean kLeadMotorInverted = false;
    public static final boolean kEncoderInverted = true;

    public static final double kP = 1;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 1;
    public static final double kMaxAccelerationRadPerSecSquared = 2;

    public static final double kPivotEncoderPositionFactor = Units.degreesToRadians(360); // rads
    public static final double kPivotEncoderVelocityFactor =
        Units.degreesToRadians(360.0) / 60.0; // rads per second

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kPivotOffsetRads = 0.5;
  }
}
