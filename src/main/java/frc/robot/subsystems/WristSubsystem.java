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
import frc.utils.TunableNumber;



public class WristSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new WristSubsystem. */

  private final CANSparkFlex WristMotor = new CANSparkFlex(WristConstants.kWristMotor, MotorType.kBrushless);

  private final AbsoluteEncoder m_encoder = WristMotor.getAbsoluteEncoder(Type.kDutyCycle);
  
  public WristSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            WristConstants.kPWristMotor,
            WristConstants.kIWristMotor,
            WristConstants.kDWristMotor,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                    WristConstants.kWristMotorMaxV, 
                    WristConstants.kWristMotorMaxA)));

    WristMotor.restoreFactoryDefaults();
    WristMotor.setIdleMode(IdleMode.kCoast);
    WristMotor.setSmartCurrentLimit(WristConstants.kWristMotorCurrentLimit);
          }


  public Command WristMotorIntakeCommand() {
          return Commands.runOnce( () -> {
              this.setGoal(WristConstants.kWristMotorIntakeGoal);
              this.enable();
          },  
              this );
          }
  

  public Command WristMotorAmpCommand() {
          return Commands.runOnce( () -> {
              this.setGoal(WristConstants.kWristMotorAmpGoal);
              this.enable();
          },  
              this );
          }

  public Command WristMotorSpeakerCommand() {
          return Commands.runOnce( () -> {
              this.setGoal(WristConstants.kWristMotorSpeakerGoal);
              this.enable();
          },  
              this );
          }

  public Command WristMotorTrapCommand() {
          return Commands.runOnce( () -> {
              this.setGoal(WristConstants.kWristMotorTrapGoal);
              this.enable();
          },  
              this );
          }


  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_encoder.getPosition();
  }
}
