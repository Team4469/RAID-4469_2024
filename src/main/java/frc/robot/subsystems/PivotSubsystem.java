// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.PivotConstants;
import frc.robot.SetPoints.PivotSetpoints;

public class PivotSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax m_leadMotor;
  private final CANSparkMax m_followMotor;
  private final AbsoluteEncoder m_encoder;
  private final ArmFeedforward m_armFeedforward =
      new ArmFeedforward(
          PivotConstants.kSVolts, PivotConstants.kGVolts,
          PivotConstants.kVVoltSecondPerRad, PivotConstants.kAVoltSecondSquaredPerRad);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            PivotConstants.kP,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                PivotConstants.kMaxVelocityRadPerSecond,
                PivotConstants.kMaxAccelerationRadPerSecSquared)));

    m_leadMotor = new CANSparkMax(PivotConstants.kLeadMotorPort, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(PivotConstants.kFollowerMotorPort, MotorType.kBrushless);

    m_encoder = m_leadMotor.getAbsoluteEncoder(Type.kDutyCycle);

    m_leadMotor.setInverted(PivotConstants.kLeadMotorInverted);

    m_leadMotor.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);

    m_leadMotor.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.kForwardSoftLimit);
    m_leadMotor.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.kReverseSoftLimit);
    m_leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_encoder.setZeroOffset(PivotConstants.kPivotOffsetRads);

    m_followMotor.follow(m_leadMotor, false);

    m_encoder.setPositionConversionFactor(PivotConstants.kPivotEncoderPositionFactor);
    m_encoder.setVelocityConversionFactor(PivotConstants.kPivotEncoderVelocityFactor);

    m_leadMotor.burnFlash();
    m_followMotor.burnFlash();

    setGoal(PivotSetpoints.kStowed);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_armFeedforward.calculate(setpoint.position, setpoint.velocity);

    // + feedforward later
    // Use the output (and optionally the setpoint) here
    m_leadMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_encoder.getPosition();
  }

  public double getRadiansFromHorizontal() {
    return m_encoder.getPosition() - PivotConstants.kPivotOffsetRads;
  }

  public Command pivotAmpSmartCommand(AmpDirection ampDirection) {
    double point;
    switch (ampDirection) {
      case FRONT:
        point = PivotSetpoints.kAmpFront;
        break;
      default:
        point = PivotSetpoints.kAmpRear;
        break;
    }
    return Commands.runOnce(
        () -> {
          this.setGoal(point);
          this.enable();
        },
        this);
  }

  public Command positionStowed() {
    return Commands.runOnce(
        () -> {
          this.setGoal(PivotSetpoints.kStowed);
          this.enable();
        },
        this);
  }

  public Command positionSubwoofer() {
    return Commands.runOnce(
        () -> {
          this.setGoal(PivotSetpoints.kSubwoofer);
          this.enable();
        },
        this);
  }

  public Command positionAmpFront() {
    return Commands.runOnce(
        () -> {
          this.setGoal(PivotSetpoints.kAmpFront);
          this.enable();
        },
        this);
  }

  public Command positionAmpRear() {
    return Commands.runOnce(
        () -> {
          this.setGoal(PivotSetpoints.kAmpRear);
          this.enable();
        },
        this);
  }

  public Command positionTrap() {
    return Commands.runOnce(
        () -> {
          this.setGoal(PivotSetpoints.kTrap);
          this.enable();
        },
        this);
  }

  public Command positionIntake() {
    return Commands.runOnce(
        () -> {
          this.setGoal(PivotSetpoints.kIntake);
          this.enable();
        },
        this);
  }
}
