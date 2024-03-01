// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberModule;

public class CLIMBER_TO_HEIGHT extends Command {
  private final ClimberModule m_leftModule;
  private final ClimberModule m_rightModule;
  private double height;
  private boolean isClimbing;

  /** Creates a new CLIMBER_TO_HEIGHT. */
  public CLIMBER_TO_HEIGHT(
      ClimberModule left, ClimberModule right, double heightMeters, Boolean isClimbingWithLoad) {
    this.m_leftModule = left;
    this.m_rightModule = right;
    this.height = heightMeters;
    this.isClimbing = isClimbingWithLoad;

    addRequirements(m_leftModule, m_rightModule);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftModule.setTargetHeight(height, isClimbing);
    m_rightModule.setTargetHeight(height, isClimbing);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_leftModule.isAtTargetPosition() && m_rightModule.isAtTargetPosition());
    // return true;
  }
}
