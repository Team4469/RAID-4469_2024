// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedExeption;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.LevetatorConstants;

public class LevetatorSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax LevetatorMotor = new CANSparkMax(LevetatorConstants.kLevetatorMotor, MotorType.kBrushless);

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
  
  LevetatorMotor.restoreFactoryDefaults();

  LevetatorMotor.setIdleMode(IdleMode.kCoast);

  LevetatorMotor.setSmartCurrentLimit(LevetatorConstants.kSmartCurrentLimit);

  LevetatorMotor.burnFlash();
  }
@Override
public void robotInit () {
  LaserCan = new LaserCan(LevetatorConstants.kLevetatorLaserCan)
  try {
    LaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
    
  }
}

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
