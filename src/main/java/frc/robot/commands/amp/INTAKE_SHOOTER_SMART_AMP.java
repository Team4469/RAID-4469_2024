// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.SetPoints.IntakeSetpoints;
import frc.robot.SetPoints.ShooterSetpoints;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class INTAKE_SHOOTER_SMART_AMP extends Command {
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private Supplier<AmpDirection> m_ampSup;
  private AmpDirection AMP_DIRECTION;
  private double shootSpeed;
  private double intakeSpeed;
  private double time;

  
  /** Creates a new INTAKE_SMART_AMP. */
  public INTAKE_SHOOTER_SMART_AMP(IntakeSubsystem intake, ShooterSubsystem shooter, Supplier<AmpDirection> amp) {
    this.m_intake = intake;
    this.m_shooter = shooter;
    this.m_ampSup = amp;

    addRequirements(m_intake, m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        AMP_DIRECTION = this.m_ampSup.get();
    if (AMP_DIRECTION == AmpDirection.FRONT) {
      intakeSpeed = IntakeSetpoints.kTransferFwdFeedSpeed;
      shootSpeed = ShooterSetpoints.kAmpFrontSpeed;
    } else {
      intakeSpeed = IntakeSetpoints.kAmpRearSpeed;
      shootSpeed = 0;
    }

    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setSpeed(intakeSpeed);
    m_shooter.setSpeed(shootSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpeed(0);
    m_shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() > (time + 1));
  }
}
