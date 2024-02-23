// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.SetPoints.PivotSetpoints;
import frc.robot.subsystems.PivotSubsystem;
import java.util.function.Supplier;

public class PIVOT_SMART_AMP extends Command {
  private PivotSubsystem m_pivot;
  private Supplier<AmpDirection> m_ampSup;
  private AmpDirection AMP_DIRECTION;
  private double setpoint;

  /** Creates a new WRIST_SMART_AMP. */
  public PIVOT_SMART_AMP(PivotSubsystem pivot, Supplier<AmpDirection> amp) {
    this.m_pivot = pivot;
    this.m_ampSup = amp;

    addRequirements(m_pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AMP_DIRECTION = this.m_ampSup.get();
    if (AMP_DIRECTION == AmpDirection.FRONT) {
      setpoint = PivotSetpoints.kAmpFront;
    } else {
      setpoint = PivotSetpoints.kAmpRear;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.setSetpoint(setpoint);
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
