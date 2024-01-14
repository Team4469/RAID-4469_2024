// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.TunableNumber;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class DRIVE_WITH_HEADING extends Command {
  // private ShuffleboardTab tab = Shuffleboard.getTab("Heading Drive");

  // GenericEntry errorEntry = tab.add("PID Controller Error", 0).withSize(2, 1).getEntry();
  // GenericEntry rotOutEntry = tab.add("Rotation Ouptut", 0).withSize(2, 1).getEntry();

  private final DriveSubsystem drive;

  Optional<Alliance> ally;
  
  private double DESIRED_HEADING_RADIANS;

  // input suppliers from joysticks
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  // Set Default Values
  private double ROTATION_PID_KP_DEFAULT = .1;
  private double ROTATION_PID_KI_DEFAULT = 0;
  private double ROTATION_PID_KD_DEFAULT = 100;
  private double ROTATION_PID_TOLERANCE_DEFAULT = 5; // DEGREES

  // Create Tuneable Numbers
  private final TunableNumber ROTATION_PID_KP =
      new TunableNumber("Swerve/DriveConstHead/kP", ROTATION_PID_KP_DEFAULT);
  private final TunableNumber ROTATION_PID_KD =
      new TunableNumber("Swerve/DriveConstHead/kD", ROTATION_PID_KD_DEFAULT);
  private final TunableNumber ROTATION_PID_TOLERANCE_DEGREES =
      new TunableNumber("Swerve/DriveConstHead/toleranceDeg", ROTATION_PID_TOLERANCE_DEFAULT);

  // Create Profiled PID Controller
  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          0.4,
          0,
          0,
          new TrapezoidProfile.Constraints(
              DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAcceleration));

  /**
   * Creates a new DriveWithHeading.
   *
   * @param Drive is the drive subsystem
   * @param xVelocity is the X translation from the driver controller
   * @param yVelocity is the Y translation from the driver controller
   * @param desiredHeading is the desired drive heading in degrees
   */
  public DRIVE_WITH_HEADING(
      DriveSubsystem Drive,
      DoubleSupplier xVelocity,
      DoubleSupplier yVelocity,
      double desiredHeading) {
    this.drive = Drive;

    this.m_translationXSupplier = xVelocity;
    this.m_translationYSupplier = yVelocity;

    this.DESIRED_HEADING_RADIANS = Units.degreesToRadians(desiredHeading);

    addRequirements(drive);

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(5));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset(drive.getPose().getRotation().getRadians());

    ally = DriverStation.getAlliance();

    if (ally.isPresent() && ally.get() == Alliance.Red) {
      DESIRED_HEADING_RADIANS = Units.degreesToRadians(180) + DESIRED_HEADING_RADIANS;
    }

    rotationController.setGoal(DESIRED_HEADING_RADIANS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotationOutput;
    if (rotationController.atGoal()) {
      rotationOutput = 0;
    } else {
      rotationOutput = rotationController.calculate(drive.getPose().getRotation().getRadians());
    }

    double xVelo =
        m_translationXSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond;
    double yVelo =
        m_translationYSupplier.getAsDouble() * DriveConstants.kMaxSpeedMetersPerSecond;

    if (ally.isPresent() && ally.get() == Alliance.Red) {
      xVelo *= -1;
      yVelo *= -1;
    }

    drive.drive(xVelo, yVelo, rotationOutput, true, true);

    // if(ROTATION_PID_KP.hasChanged()) {
    //   rotationController.setP(ROTATION_PID_KP.get());
    // }
    // if(ROTATION_PID_KD.hasChanged()) {
    //   rotationController.setD(ROTATION_PID_KD.get());
    // }
    // if(ROTATION_PID_TOLERANCE_DEGREES.hasChanged()) {
    //
    // rotationController.setTolerance(Units.degreesToRadians(ROTATION_PID_TOLERANCE_DEGREES.get()));
    // }
    // SmartDashboard.putNumber("PID Error", rotationController.getPositionError());
    // SmartDashboard.putNumber("Rotation Output", rotationOutput);
    // errorEntry.setDouble(rotationController.getPositionError());
    // rotOutEntry.setDouble(rotationOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
