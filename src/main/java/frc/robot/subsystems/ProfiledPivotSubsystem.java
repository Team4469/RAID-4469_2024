// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Setpoints.LevetatorPivotSetpoints;
import frc.robot.Constants.LevetatorPivotConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.util.Units;


public class ProfiledPivotSubsystem extends ProfiledPIDSubsystem {
  
  private final CANSparkFlex m_leadMotor = new CANSparkFlex(LevetatorPivotConstants.kLPLeadMotorCanId, MotorType.kBrushless);
  private final CANSparkFlex m_followMotor = new CANSparkFlex(LevetatorPivotConstants.kLPFollowMotorCanId, MotorType.kBrushless);
  private final AbsoluteEncoder m_encoder = m_leadMotor.getAbsoluteEncoder(Type.kDutyCycle);
  
  /** Creates a new LevatatorPivotPID. */
  public ProfiledPivotSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            LevetatorPivotConstants.kP,
            LevetatorPivotConstants.kI,
            LevetatorPivotConstants.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(LevetatorPivotConstants.kMinOutput, LevetatorPivotConstants.kMaxOutput)));
      

      
      m_leadMotor.restoreFactoryDefaults();
      m_followMotor.restoreFactoryDefaults();
  
      m_leadMotor.setIdleMode(IdleMode.kBrake);
      m_followMotor.setIdleMode(IdleMode.kBrake);

      m_leadMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRadians(85));
      m_leadMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.degreesToRadians(85+115));
      m_leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      m_leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

      m_encoder.setZeroOffset(LevetatorPivotConstants.kPivotOffsetRads);

      m_leadMotor.setSmartCurrentLimit(LevetatorPivotConstants.kSmartCurrentLimit);
      m_followMotor.setSmartCurrentLimit(LevetatorPivotConstants.kSmartCurrentLimit);
  
      m_followMotor.follow(m_leadMotor, false);
      m_leadMotor.setInverted(LevetatorPivotConstants.kLeadMotorInverted);

      m_encoder.setInverted(LevetatorPivotConstants.kEncoderInverted);
      m_encoder.setPositionConversionFactor(LevetatorPivotConstants.kPositionConversionFactor);
      m_encoder.setVelocityConversionFactor(LevetatorPivotConstants.kVelocityConversionFactor);
  
      m_leadMotor.burnFlash();
      m_followMotor.burnFlash(); 

      setGoal(LevetatorPivotSetpoints.kStowGoal);

        
  }


    // Create variables
    //private final CANSparkFlex m_leadMotor;
    //private final CANSparkFlex m_followMotor;

  
    public void MotorsOn(double speed){
      if(Math.abs(speed) > 1) {speed = 1 * Math.signum(speed);}
      m_leadMotor.set(speed);
    }
  
    public void MotorsOff(){
      m_leadMotor.set(0);
    }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_leadMotor.setVoltage(output);
  }


  @Override
  public double getMeasurement() {
    // Return the process variable measurement her
    return m_encoder.getPosition();
  }

   public Command LPRotationGoalCommand(){
    return Commands.runOnce(
      () -> {
            this.setGoal(LevetatorPivotSetpoints.kLPRotationGoal);
            this.enable();
      },
    this);
  }

  public Command StowPositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorPivotSetpoints.kStowGoal);
        this.enable();
      },
    this);
  }

    public Command IntakePositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorPivotSetpoints.kIntakeGoal);
        this.enable();
      },
    this);
  }

    public Command SubwooferPositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorPivotSetpoints.kSubwooferGoal);
        this.enable();
      },
    this);
  }

    public Command AmpForwardPositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorPivotSetpoints.kAmpForwardGoal);
        this.enable();
      },
    this);
  }

    public Command AmpBackPositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorPivotSetpoints.kAmpBackGoal);
        this.enable();
      },
    this);
  }

    public Command TrapPositionCommand() {
    return Commands.runOnce(
      () -> {
        this.setGoal(LevetatorPivotSetpoints.kTrapGoal);
        this.enable();
      },
    this);
  }


}
