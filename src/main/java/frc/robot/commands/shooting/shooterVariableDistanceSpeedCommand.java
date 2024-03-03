// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.utils.ShootingInterpolationTables.ShooterLaunchAngleTable;
import frc.utils.ShootingInterpolationTables.ShooterSpeedTable;
import java.util.function.DoubleSupplier;

public class shooterVariableDistanceSpeedCommand extends Command {
  private ShooterSubsystem m_shoot;
  private WristSubsystem m_wrist;
  private DoubleSupplier m_distSup;
  private double distanceMeters;
  private double shooterTarget;
  private double wristTarget;

  /** Creates a new shooterVariableDistanceSpeedCommand. */
  public shooterVariableDistanceSpeedCommand(
      ShooterSubsystem shoot, WristSubsystem wrist, DoubleSupplier distance) {
    this.m_shoot = shoot;
    this.m_wrist = wrist;
    this.m_distSup = distance;

    this.distanceMeters = m_distSup.getAsDouble();
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_shoot, m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceMeters = m_distSup.getAsDouble();
    shooterTarget = ShooterSpeedTable.SHOOTER_SPEED_INTERP_TABLE.get(distanceMeters);
    wristTarget = ShooterLaunchAngleTable.SHOOTER_LAUNCH_ANGLE_INTERP_TABLE.get(distanceMeters);

    m_wrist.setSetpoint(wristTarget);
    m_shoot.setSpeed(shooterTarget);
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
