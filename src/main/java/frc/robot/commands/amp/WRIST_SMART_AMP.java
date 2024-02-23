// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.SetPoints.WristSetpoints;
import frc.robot.subsystems.WristSubsystem;
import java.util.function.Supplier;

public class WRIST_SMART_AMP extends Command {
  private WristSubsystem m_wrist;
  private Supplier<AmpDirection> m_ampSup;
  private AmpDirection AMP_DIRECTION;
  private double setpoint;

  /** Creates a new WRIST_SMART_AMP. */
  public WRIST_SMART_AMP(WristSubsystem wrist, Supplier<AmpDirection> amp) {
    this.m_wrist = wrist;
    this.m_ampSup = amp;

    addRequirements(m_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AMP_DIRECTION = this.m_ampSup.get();
    if (AMP_DIRECTION == AmpDirection.FRONT) {
      setpoint = WristSetpoints.kAmpFront;
    } else {
      setpoint = WristSetpoints.kAmpRear;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setSetpoint(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
