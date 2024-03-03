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
import frc.robot.subsystems.utils.Limelight;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class STAGE_ALIGN_DRIVE extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private double heading;
  private boolean validTarget;
  private final Limelight m_ll;

  private int targetSeen;

  private Optional<Alliance> ally;

  private double xSpeed;
  private double ySpeed;

  private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          .4,
          0,
          0,
          new TrapezoidProfile.Constraints(
              DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAcceleration));

  /** H */
  public STAGE_ALIGN_DRIVE(
      DriveSubsystem Drive, DoubleSupplier xVelocity, DoubleSupplier yVelocity, Limelight ll) {

    this.m_drive = Drive;
    this.m_translationXSupplier = xVelocity;
    this.m_translationYSupplier = yVelocity;
    this.m_ll = ll;

    addRequirements(this.m_drive, this.m_ll);

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(5));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetSeen = (int) m_ll.getTargetInView();
    switch (targetSeen) {
      case 11:
        heading = 120;
        break;
      case 12:
        heading = 240;
        break;
      case 13:
        heading = 0;
        break;
      case 14:
        heading = 180;
        break;
      case 15:
        heading = 300;
        break;
      case 16:
        heading = 60;
        break;
      default:
        heading = m_drive.getHeading().getDegrees();
        break;
    }

    rotationController.reset(m_drive.getPose().getRotation().getRadians());

    rotationController.setGoal(Units.degreesToRadians(heading));

    ally = DriverStation.getAlliance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    validTarget = m_ll.hasTargets();

    if (validTarget) {
      xSpeed = m_ll.limelight_strafe_x_proportional();
      ySpeed = m_ll.limelight_strafe_y_proportional();

      // if (ally.isPresent() && ally.get() == Alliance.Red) {

        xSpeed *= -1.0;
        ySpeed *= -1.0;
      // }

    } else {
      xSpeed = this.m_translationXSupplier.getAsDouble();
      ySpeed = this.m_translationYSupplier.getAsDouble();
    }

    double rotationOutput;
    if (rotationController.atGoal()) {
      rotationOutput = 0;
    } else {
      rotationOutput = rotationController.calculate(m_drive.getPose().getRotation().getRadians());
    }

    double xVelo = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double yVelo = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

    m_drive.drive(xVelo, yVelo, rotationOutput, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ll.limelight_in_range();
  }
}
