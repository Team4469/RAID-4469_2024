// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.LevetatorConstants;
import frc.robot.Setpoints.LevetatorSetPoints;

public class LevetatorSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax m_Motor;
  private LaserCan m_laserCan;


  /** Creates a new LevetatorSubsystem. */
  public LevetatorSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            LevetatorConstants.kP,
            LevetatorConstants.kI,
            LevetatorConstants.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(LevetatorConstants.kMaxVelocityMetersPerSec, LevetatorConstants.kMaxAccelerationMetersPerSecSquared)));

   m_laserCan = new LaserCan(LevetatorConstants.kLaserCanId); 

  m_Motor = new CANSparkMax(LevetatorConstants.kMotorCanId, MotorType.kBrushless);
  m_Motor.restoreFactoryDefaults();
  m_Motor.setIdleMode(IdleMode.kCoast);
  m_Motor.setSmartCurrentLimit(LevetatorConstants.kSmartCurrentLimit);
  m_Motor.burnFlash();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_Motor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
   return m_laserCan.getMeasurement().distance_mm/1000;
  }

  public Command LevetatorStowPositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorSetPoints.kStowGoal);
        this.enable();
      },
      this);
  }

    public Command LevetatorIntakePositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorSetPoints.kIntakeGoal);
        this.enable();
      },
      this);
  }

    public Command LevetatorForwardAmpPositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorSetPoints.kAmpForwardGoal);
        this.enable();
      },
      this);
  }

    public Command LevetatorSubwooferPositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorSetPoints.kSubwooferGoal);
        this.enable();
      },
      this);
  }

    public Command LevetatorTrapPositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorSetPoints.kTrapGoal);
        this.enable();
      },
      this);
  }
}
