// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.utils.Limelight;
import java.util.function.DoubleSupplier;

public class AMP_ALIGN_DRIVE extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final Limelight m_ll;
  private AmpDirection AMP_DIR;
  private double heading;
  private boolean validTarget;

  private double xSpeed;
  private double ySpeed;

    private ProfiledPIDController rotationController =
      new ProfiledPIDController(
          .4,
          0,
          0,
          new TrapezoidProfile.Constraints(
              DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAcceleration));


  /**
   * 
   * @param Drive
   * @param xVelocity
   * @param yVelocity
   * @param amp
   * @param ll
   */
  public AMP_ALIGN_DRIVE(
      DriveSubsystem Drive,
      DoubleSupplier xVelocity,
      DoubleSupplier yVelocity,
      AmpDirection amp,
      Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = Drive;
    this.m_translationXSupplier = xVelocity;
    this.m_translationYSupplier = yVelocity;
    this.AMP_DIR = amp;
    this.m_ll = ll;

    addRequirements(this.m_drive, this.m_ll);

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(5));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (AMP_DIR == AmpDirection.FRONT) {
      heading = 90;
    } else {
      heading = 270;
    }

    rotationController.reset(m_drive.getPose().getRotation().getRadians());


    rotationController.setGoal(Units.degreesToRadians(heading));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    validTarget = m_ll.hasTargets();

    if (validTarget) {
      xSpeed = m_ll.limelight_strafe_x_proportional();
      ySpeed = m_ll.limelight_strafe_y_proportional();
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
