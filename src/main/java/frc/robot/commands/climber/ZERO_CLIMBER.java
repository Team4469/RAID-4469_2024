// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberModule;

public class ZERO_CLIMBER extends Command {
  private static final double CLIMBER_ZERO_VELOCITY_TIME_PERIOD = 0.5;
  private static final double REVERSE_VOLTAGE = -0.75;
  private static final double VELOCITY_THRESHOLD = 0.00254 / 60.0;

  private final ClimberModule climber;

  private double zeroVelocityTimestamp;

  /** Creates a new ZERO_CLIMBER. */
  public ZERO_CLIMBER(ClimberModule climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    zeroVelocityTimestamp = Double.NaN;
    climber.setZeroed(false);
  }

  @Override
  public void execute() {
    climber.setTargetVoltage(REVERSE_VOLTAGE);
    if (Math.abs(climber.getCurrentVelocity()) < VELOCITY_THRESHOLD) {
      if (!Double.isFinite(zeroVelocityTimestamp)) {
        zeroVelocityTimestamp = Timer.getFPGATimestamp();
      }
    } else {
      zeroVelocityTimestamp = Double.NaN;
    }
  }

  @Override
  public boolean isFinished() {
    if (Double.isFinite(zeroVelocityTimestamp)) {
      return Timer.getFPGATimestamp() - zeroVelocityTimestamp >= CLIMBER_ZERO_VELOCITY_TIME_PERIOD;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climber.setTargetVoltage(0.0);
    if (!interrupted) {
      climber.setZeroPosition();
      climber.setTargetHeight(0, false);
      climber.setZeroed(true);
    }
  }
}
