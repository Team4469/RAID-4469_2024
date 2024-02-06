// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.FrontWristConstants;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.SetPoints.WristSetpoints;

public class WristSubsystem extends TrapezoidProfileSubsystem {
  private final CANSparkFlex m_wristMotor =
      new CANSparkFlex(FrontWristConstants.kFrontWristMotorID, MotorType.kBrushless);

  private final SparkPIDController m_wristPIDController = m_wristMotor.getPIDController();

  private final AbsoluteEncoder m_wristAbsoluteEncoder =
      m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 2.98, 0.02);

  /** Creates a new FrontWristIntake. */
  public WristSubsystem() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(
            FrontWristConstants.kMaxVelocityRadPerSecond,
            FrontWristConstants.kMaxAccelerationRadPerSecSquared),
        // The initial position of the mechanism
        FrontWristConstants.kStowedRads);

    m_wristMotor.restoreFactoryDefaults();

    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setSmartCurrentLimit(FrontWristConstants.kSmartCurrentLimit);

    m_wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) FrontWristConstants.kMaxRads);
    m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) FrontWristConstants.kMinRads);

    m_wristMotor.burnFlash();

    m_wristAbsoluteEncoder.setPositionConversionFactor(2.0 * Math.PI);
    m_wristAbsoluteEncoder.setVelocityConversionFactor((2.0 * Math.PI) / 60.0);

    m_wristPIDController.setFeedbackDevice(m_wristAbsoluteEncoder);
    m_wristPIDController.setPositionPIDWrappingEnabled(false);
    m_wristPIDController.setPositionPIDWrappingMaxInput(FrontWristConstants.kMaxRads);
    m_wristPIDController.setPositionPIDWrappingMinInput(FrontWristConstants.kMinRads);
    m_wristPIDController.setP(0.000001, 0);
    m_wristPIDController.setI(0, 0);
    m_wristPIDController.setD(0, 0);
  }

  @Override
  protected void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_wristPIDController.setReference(
        setpoint.position, ControlType.kPosition, 0, feedforward / 12.0);
  }

  public Command wristAmpSmartCommand(AmpDirection ampDirection) {
    double point;
    switch (ampDirection) {
      case FRONT:
        point = WristSetpoints.kAmpFront;
        break;
      default:
        point = WristSetpoints.kAmpRear;
        break;
    }
    return Commands.runOnce(
        () -> {
          this.setGoal(point);
          this.enable();
        },
        this);
  }

  public Command positionIntake() {
    return Commands.runOnce(
        () -> {
          this.setGoal(WristSetpoints.kIntake);
          this.enable();
        },
        this);
  }

  public Command positionStowed() {
    return Commands.runOnce(
        () -> {
          this.setGoal(WristSetpoints.kStowed);
          this.enable();
        },
        this);
  }

  public Command positionSubwoofer() {
    return Commands.runOnce(
        () -> {
          this.setGoal(WristSetpoints.kSubwoofer);
          this.enable();
        },
        this);
  }

  public Command positionAmpFront() {
    return Commands.runOnce(
        () -> {
          this.setGoal(WristSetpoints.kAmpFront);
          this.enable();
        },
        this);
  }

  public Command positionAmpRear() {
    return Commands.runOnce(
        () -> {
          this.setGoal(WristSetpoints.kAmpRear);
          this.enable();
        },
        this);
  }

  public Command positionTrap() {
    return Commands.runOnce(
        () -> {
          this.setGoal(WristSetpoints.kTrap);
          this.enable();
        },
        this);
  }
}
