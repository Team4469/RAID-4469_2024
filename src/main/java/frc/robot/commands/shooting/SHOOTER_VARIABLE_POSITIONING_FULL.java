// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import static frc.utils.ShootingInterpolationTables.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import java.util.function.DoubleSupplier;

public class SHOOTER_VARIABLE_POSITIONING_FULL extends Command {
  private ShooterSubsystem m_shoot;
  private WristSubsystem m_wrist;
  private PivotSubsystem m_pivot;
  private DoubleSupplier m_distSup;
  private double distanceMeters;
  private double shooterTarget;
  private double wristTarget;
  private double pivotTarget;

  /** Creates a new SHOOTER_VARIABLE_POSITIONING_FULL. */
  public SHOOTER_VARIABLE_POSITIONING_FULL(
      ShooterSubsystem shoot,
      WristSubsystem wrist,
      PivotSubsystem pivot,
      DoubleSupplier distanceSupplier) {
    this.m_shoot = shoot;
    this.m_wrist = wrist;
    this.m_pivot = pivot;
    this.m_distSup = distanceSupplier;

    this.distanceMeters = m_distSup.getAsDouble();

    addRequirements(m_shoot, m_wrist, m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceMeters = m_distSup.getAsDouble();
    shooterTarget = SHOOTER_SPEED_INTERP_TABLE.get(distanceMeters);
    wristTarget = WRIST_LAUNCH_ANGLE_INTERP_TABLE.get(distanceMeters);
    pivotTarget = PIVOT_SHOOTER_ANGLE_INTERP_TABLE.get(distanceMeters);

    m_wrist.setSetpoint(wristTarget);
    m_shoot.setSpeed(shooterTarget);
    m_pivot.setSetpoint(pivotTarget);
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
