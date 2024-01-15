// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import frc.utils.TunableNumber;

public class DriveSubsystem extends SubsystemBase {
  // Logging
  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
  private GenericEntry gyroEntry =
      driveTab.add("Gyro", 0).withWidget(BuiltInWidgets.kGyro).getEntry();

  private GenericEntry flRealAngle = driveTab.add("FL Real Angle", 0).getEntry();
  private GenericEntry frRealAngle = driveTab.add("FR Real Angle", 0).getEntry();
  private GenericEntry rlRealAngle = driveTab.add("RL Real Angle", 0).getEntry();
  private GenericEntry rrRealAngle = driveTab.add("RR Real Angle", 0).getEntry();

  private GenericEntry flDesiredAngle = driveTab.add("FL Desired Angle", 0).getEntry();
  private GenericEntry frDesiredAngle = driveTab.add("FR Desired Angle", 0).getEntry();
  private GenericEntry rlDesiredAngle = driveTab.add("RL Desired Angle", 0).getEntry();
  private GenericEntry rrDesiredAngle = driveTab.add("RR Desired Angle", 0).getEntry();

  private GenericEntry flRealSpeed = driveTab.add("FL Real Speed", 0).getEntry();
  private GenericEntry frRealSpeed = driveTab.add("FR Real Speed", 0).getEntry();
  private GenericEntry rlRealSpeed = driveTab.add("RL Real Speed", 0).getEntry();
  private GenericEntry rrRealSpeed = driveTab.add("RR Real Speed", 0).getEntry();

  private GenericEntry flDesiredSpeed = driveTab.add("FL Desired Speed", 0).getEntry();
  private GenericEntry frDesiredSpeed = driveTab.add("FR Desired Speed", 0).getEntry();
  private GenericEntry rlDesiredSpeed = driveTab.add("RL Desired Speed", 0).getEntry();
  private GenericEntry rrDesiredSpeed = driveTab.add("RR Desired Speed", 0).getEntry();

  private double ANGULAR_VELOCITY_COEFFICIENT_DEFAULT = 0.01;

  TunableNumber ANGULAR_VELOCITY_COEFFICIENT =
      new TunableNumber("Drive/AngularVeloCoefficient", ANGULAR_VELOCITY_COEFFICIENT_DEFAULT);

  TunableNumber DRIVE_SPEED_MULTIPLIER = new TunableNumber("Drive/SpeedMultiplier", 1);

  TunableNumber ROTATION_SPEED_MULTIPLIER = new TunableNumber("Drive/RotationMultiplier", 1);

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro =
      new AHRS(SPI.Port.kMXP); // navX Gyroscope is plugged in to the rio SPI MXP Port

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(getAngleCorrected()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    AutoBuilder.configureHolonomic(
        this::getPose, 
        this::resetOdometry,
        this::getRobotRelativeSpeeds, 
        this::driveRobotRelative, 
        new HolonomicPathFollowerConfig(
          new PIDConstants(5.0, 0, 0), 
          new PIDConstants(5.0, 0, 0), 
          DriveConstants.kMaxSpeedMetersPerSecond,
          DriveConstants.kRobotDriveRadiusMeters,
          new ReplanningConfig()), 
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
        this);
  }

  // COMMANDS
  public Command setXCommand(){
    return this.runOnce(this::setX);
  };

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getAngleCorrected()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    // LOGGING
    gyroEntry.setDouble(m_gyro.getAngle());

    flRealAngle.setDouble(m_frontLeft.getState().angle.getDegrees());
    frRealAngle.setDouble(m_frontRight.getState().angle.getDegrees());
    rlRealAngle.setDouble(m_rearLeft.getState().angle.getDegrees());
    rrRealAngle.setDouble(m_rearRight.getState().angle.getDegrees());

    flRealSpeed.setDouble(m_frontLeft.getState().speedMetersPerSecond);
    frRealSpeed.setDouble(m_frontRight.getState().speedMetersPerSecond);
    rlRealSpeed.setDouble(m_rearLeft.getState().speedMetersPerSecond);
    rrRealSpeed.setDouble(m_rearRight.getState().speedMetersPerSecond);
  }

  public Command zeroGyro() {
    return this.runOnce(this::zeroHeading);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    xSpeed *= DRIVE_SPEED_MULTIPLIER.get();
    ySpeed *= DRIVE_SPEED_MULTIPLIER.get();
    rot *= ROTATION_SPEED_MULTIPLIER.get();

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(getAngleCorrected())
                        .plus(
                            new Rotation2d(
                                this.getAngularVelocity() * ANGULAR_VELOCITY_COEFFICIENT.get())))
                // Testing this to correct for drift while driving and rotating
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    // LOGGING
    flDesiredAngle.setDouble(swerveModuleStates[0].angle.getDegrees());
    frDesiredAngle.setDouble(swerveModuleStates[1].angle.getDegrees());
    rlDesiredAngle.setDouble(swerveModuleStates[2].angle.getDegrees());
    rrDesiredAngle.setDouble(swerveModuleStates[3].angle.getDegrees());

    flDesiredSpeed.setDouble(swerveModuleStates[0].speedMetersPerSecond);
    frDesiredSpeed.setDouble(swerveModuleStates[1].speedMetersPerSecond);
    rlDesiredSpeed.setDouble(swerveModuleStates[2].speedMetersPerSecond);
    rrDesiredSpeed.setDouble(swerveModuleStates[3].speedMetersPerSecond);
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngleCorrected()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getAngleCorrected() {
    return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getAngularVelocity() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
    };
  }

}
