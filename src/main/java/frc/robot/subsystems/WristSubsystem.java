// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants.WristConstants;
import frc.robot.Setpoints.WristSetpoints;



public class WristSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new WristSubsystem. */

  private final CANSparkFlex WristMotor;
  private final AbsoluteEncoder WristEncoder;
    
  public WristSubsystem() {

    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            WristConstants.kP,
            WristConstants.kI,
            WristConstants.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                    WristConstants.kMaxVelocityRadiansPerSecond, 
                    WristConstants.kMaxAccelerationMetersPerSecondSquared)));

    WristMotor = new CANSparkFlex(WristConstants.kWristMotorCanid, MotorType.kBrushless);
    WristMotor.restoreFactoryDefaults();
    WristMotor.setIdleMode(IdleMode.kCoast);
    WristMotor.setSmartCurrentLimit(WristConstants.kCurrentLimit);

    WristEncoder = WristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    WristEncoder.setPositionConversionFactor(WristConstants.kPositionConversionFactor);
    WristEncoder.setVelocityConversionFactor(WristConstants.kVelocityConversionFactor);
}


  public Command WristIntakePositionCommand() {
          return Commands.runOnce( () -> {
              this.setGoal(WristSetpoints.kIntakeGoal);
              this.enable();
          },  
              this );
          }
  

  public Command WristAmpPositionCommand() {
          return Commands.runOnce( () -> {
              this.setGoal(WristSetpoints.kAmpGoal);
              this.enable();
          },  
              this );
          }

  public Command WristSpeakerPositionCommand() {
          return Commands.runOnce( () -> {
              this.setGoal(WristSetpoints.kSpeakerGoal);
              this.enable();
          },  
              this );
          }

  public Command WristTrapPositionCommand() {
          return Commands.runOnce( () -> {
              this.setGoal(WristSetpoints.kTrapGoal);
              this.enable();
          },  
              this );
          }


  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
  // Use the output (and optionally the setpoint) here
    WristMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return WristEncoder.getPosition();
  }
}
