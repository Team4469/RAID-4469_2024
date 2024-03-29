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
    public static final int kOperatorController2Port = 2;
    public static final double kDriveDeadband = 0.02;

    public static final class TopButtons {
      public static final int CLIMB_UP = 1;
      public static final int CLIMB_DOWN = 6;
      public static final int TRAP_PREP = 2;
      public static final int TRAP_EXT = 4;
      public static final int CLIMB_TRAP = 8;
      public static final int CLIMB_HARM = 5;
      public static final int TRAP_OUTTAKE = 3;
      public static final int AUTO_TRAP = 7;
    }

    public static final class BottomButtons {
      public static final int SHOOTER_ON = 3;
      public static final int SHOOTER_OFF = 1;
      public static final int OUTTAKE = 2;
      public static final int SUBWOOFER_ON = 4;
    }
  }

  public static final class GlobalConstants {
    public enum AmpDirection {
      FRONT,
      REAR
    }

    public enum StageLocationAlliance {
      STAGE_LEFT_BLUE,
      STAGE_RIGHT_BLUE,
      CENTER_STAGE_BLUE,
      STAGE_LEFT_RED,
      STAGE_RIGHT_RED,
      CENTER_STAGE_RED
    }

    public enum StageLoc {
      STAGE_LEFT,
      STAGE_RIGHT,
      CENTER_STAGE
    }
  }

  public static final class PathFollowingConstraints {
    public static final PathConstraints kStagePathConstraints =
        new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(540));
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.7;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAngularAcceleration =
        kMaxAngularSpeed * 1.5; // FIND Actual number

    public static final double kDirectionSlewRate = 2; // radians per second
    public static final double kMagnitudeSlewRate = 2.2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.5; // percent per second (1 = 100%)

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

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kFrontLeftTurningCanId = 11;

    public static final int kFrontRightDrivingCanId = 18;
    public static final int kFrontRightTurningCanId = 13;

    public static final int kRearRightDrivingCanId = 14;
    public static final int kRearRightTurningCanId = 15;

    public static final int kRearLeftDrivingCanId = 16;
    public static final int kRearLeftTurningCanId = 17;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14; // High Speed

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
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class LeftClimberConstants {
    public static final int kMotorID = 36;
    public static final int kSensorID = 3;
    public static final boolean kMotorInverted = false;

    public static final double kP_Climbing = 32;
    public static final double kI_Climbing = 0;
    public static final double kD_Climbing = 1;

    public static final double kP_No_Climbing = 4; // 4
    public static final double kI_No_Climbing = 0;
    public static final double kD_No_Climbing = 10; // 10
  }

  public static final class RightClimberConstants {
    public static final int kMotorID = 35;
    public static final int kSensorID = 0;
    public static final boolean kMotorInverted = true;

    public static final double kP_Climbing = 32;
    public static final double kI_Climbing = 0;
    public static final double kD_Climbing = 1;

    public static final double kP_No_Climbing = 3; // 3
    public static final double kI_No_Climbing = 0;
    public static final double kD_No_Climbing = 10; // 10
  }

  public static final class LevetatorConstants {
    public static final int kLevetatorMotorID = 23;
    public static final int kLevetatorLaserCanID = 50;

    public static final int kCurrentLimit = 65;

    public static final boolean kMotorInverted = true;

    public static final double kLevetatorOffset = Units.inchesToMeters(0);

    public static final double kGearRatio = 25;
    public static final double kSprocketDiameterMeters = Units.inchesToMeters(1.432);

    public static final double kPositionConversionFactor =
        kSprocketDiameterMeters * Math.PI * (1.0 / kGearRatio);
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final double kForwardSoftLimit = Units.inchesToMeters(10);
    public static final double kReverseSoftLimit = Units.inchesToMeters(0);

    public static final double kMaxVelocityMeterPerSecond = .5; // m/s
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.25; // m/s^2
    public static final double kP = 7;
    public static final double kI = 0.0;
    public static final double kD = 1;
    public static final double kS = 0;
    public static final double kGravity = 0.2;
    public static final double kV = 0;

    public static final double kClosedLoopRampRate = 1;

    public static final int kStatus3PeriodMs = 500;
    public static final int kStatus4PeriodMs = 500;
    public static final int kStatus5PeriodMs = 20;
  }

  public static final class VisionConstants {
    public static final String kFrontLimelightName = "limelight-front";
    public static final String kRearLimelightName = "limelight-rear";
  }

  public static final class FrontLimelightConstants {
    public static final double kAngleFromVerticalDegrees = 30;
    public static final double kDistanceFromFloorMeters = Units.inchesToMeters(7.15);
  }

  public static final class ClimberConstants {
    public static final double kMaxVelocity = 0.25; // mm/s
    public static final double kMaxAcceleration = 0.25; // mm/s^2
    public static final double kP = .0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double extensionSpeed = .1;
    public static final double retractionSpeed = -.5;

    public static final double kGearRatio = 15.60;

    public static final double kPositionConversionFactor =
        Units.inchesToMeters(1.18) * Math.PI * (1.0 / kGearRatio);

    public static final double kSensorOffset = Units.inchesToMeters(2.56);

    public static final int kStatus3PeriodMs = 500;
    public static final int kStatus4PeriodMs = 500;
    public static final int kStatus5PeriodMs = 500;
    public static final int kStatus6PeriodMs = 500;
  }

  public static final class ShooterConstants {
    public static final int kLeftShooterCanID = 48;
    public static final int kRightShooterCanID = 20;
    public static final boolean kLeftMotorInverted = false;
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

    public static final int kStatus3PeriodMs = 500;
    public static final int kStatus4PeriodMs = 500;
    public static final int kStatus5PeriodMs = 500;
    public static final int kStatus6PeriodMs = 500;
  }

  public static final class IntakeConstants {
    public static final int kMotorID = 22;
    public static final int kCurrentLimit = 80;

    public static final int kIntakeForwardLaserCanID = 1;
    public static final int kIntakeRearLaserCanID = 4;

    public static final int kDetectionDistanceMM = 25;
    public static final boolean kMotorInverted = false;

    public static final int kStatus3PeriodMs = 500;
    public static final int kStatus4PeriodMs = 500;
    public static final int kStatus5PeriodMs = 500;
    public static final int kStatus6PeriodMs = 500;

    public static final double kSensorDebounceTime = 0.06;
    public static final double kAutoIntakeBackMoveTime = .2;
    public static final double kShootFeedTime = .4;
  }

  public static final class PivotConstants {
    public static final int kLeadMotorPort = 30;
    public static final int kFollowerMotorPort = 31;

    public static final int kMotorCurrentLimit = 85;

    public static final boolean kLeadMotorInverted = false;
    public static final boolean kFollowMotorInverted = false;
    public static final boolean kEncoderInverted = true;

    public static final double kP = .65;
    public static final double kI = .002;
    public static final double kD = 7;
    public static final double kIz = .05;
    public static final double kIAcum = .01;

    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final double kClosedLoopRampRate = 1;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = .25;
    public static final double kMaxAccelerationRadPerSecSquared = .1;

    public static final double kPivotEncoderPositionFactor = Units.degreesToRadians(360); // rads
    public static final double kPivotEncoderVelocityFactor =
        Units.degreesToRadians(360.0) / 60.0; // rads per second

    public static final float kForwardSoftLimit = 4; // rads
    public static final float kReverseSoftLimit = (float) Units.degreesToRadians(85);

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kPivotOffsetRads = Units.degreesToRadians(90);

    public static final double kPivotZeroOffset = 5.07;
    public static final int kLeaderStatus0PeriodMs = 5;
    public static final int kFollowerStatus0PeriodMs = 100;
    public static final int kStatus3PeriodMs = 500;
    public static final int kStatus4PeriodMs = 500;
    public static final int kLeaderStatus5PeriodMs = 20;
    public static final int kFollowerStatus5PeriodMs = 500;
  }

  public static final class WristConstants {
    public static final int kWristMotorID = 50;

    public static final double kMaxVelocityRadPerSecond = 1; // rad/sec
    public static final double kMaxAccelerationRadPerSecSquared = 1; // rad/sec/sec
    public static final double kP = .33; //.33
    public static final double kI = 0.002;
    public static final double kD = 5;
    public static final double kIz = 0.05;
    public static final double kIAcum = .02;

    public static final int kSmartCurrentLimit = 115; // A

    public static final double kEncoderZeroOffset = 2.688;

    public static final boolean kEncoderInverted = false;

    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final double kPositionConversionFactor = 2.0 * Math.PI;
    public static final double kVelocityConversionFactor = (2.0 * Math.PI) / 60.0;

    public static final double kMinRads = Units.degreesToRadians(90);
    public static final double kMaxRads = Units.degreesToRadians(315);

    public static final int kStatus3PeriodMs = 500;
    public static final int kStatus4PeriodMs = 500;
    public static final int kStatus5PeriodMs = 20;
  }
}
