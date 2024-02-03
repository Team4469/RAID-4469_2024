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

public class LevetatorSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax LevetatorMotor;
  private LaserCan LevetatorLaserCan;


  /** Creates a new LevetatorSubsystem. */
  public LevetatorSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            LevetatorConstants.kP,
            LevetatorConstants.kI,
            LevetatorConstants.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(LevetatorConstants.kMaxVelocity, LevetatorConstants.kMaxAcceleration)));
  
  LevetatorLaserCan = new LaserCan(LevetatorConstants.kLaserCan); 

  LevetatorMotor = new CANSparkMax(LevetatorConstants.kLevetatorMotor, MotorType.kBrushless);
  LevetatorMotor.restoreFactoryDefaults();
  LevetatorMotor.setIdleMode(IdleMode.kCoast);
  LevetatorMotor.setSmartCurrentLimit(LevetatorConstants.kSmartCurrentLimit);
  LevetatorMotor.burnFlash();
  }

  public Command LevetatorStowCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorConstants.kStowGoal);
        this.enable();
      },
    this );
  }

  public Command LevetatorIntakeCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorConstants.kIntakeGoal);
        this.enable();
      },
    this );
  }

  public Command LevetatorAmpCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorConstants.kAmpGoal);
        this.enable();
      },
    this );
  }

  public Command LevetatorSpeakerCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorConstants.kSpeakerGoal);
        this.enable();
      },
    this );
  }

  public Command LevetatorTrapCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorConstants.kTrapGoal);
        this.enable();
      },
    this );
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    LevetatorMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
   return LevetatorLaserCan.getMeasurement().distance_mm;
  }
}
