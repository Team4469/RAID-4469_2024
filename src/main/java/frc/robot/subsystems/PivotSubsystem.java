// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.PivotConstants;
import frc.robot.SetPoints.PivotSetpoints;

public class PivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_leadMotor;
  private final CANSparkMax m_followMotor;
  private final AbsoluteEncoder m_encoder;

  private final SparkPIDController m_pidController;

  /** Creates a new PivotSubsystem2. */
  public PivotSubsystem() {

    m_leadMotor = new CANSparkMax(PivotConstants.kLeadMotorPort, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(PivotConstants.kFollowerMotorPort, MotorType.kBrushless);

    m_leadMotor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();

    m_pidController = m_leadMotor.getPIDController();

    m_pidController.setP(.35);
    m_pidController.setI(0);
    m_pidController.setD(20);

    m_pidController.setPositionPIDWrappingEnabled(false);

    m_pidController.setPositionPIDWrappingMaxInput(4);
    m_pidController.setPositionPIDWrappingMinInput(Math.PI / 2);

    m_pidController.setOutputRange(-1, 1);

    m_leadMotor.setClosedLoopRampRate(1);

    m_encoder = m_leadMotor.getAbsoluteEncoder(Type.kDutyCycle);

    m_pidController.setFeedbackDevice(m_encoder);

    m_leadMotor.setInverted(PivotConstants.kLeadMotorInverted);

    m_leadMotor.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);
    m_followMotor.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);

    m_leadMotor.setSoftLimit(SoftLimitDirection.kForward, 4);
    m_leadMotor.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.kReverseSoftLimit);
    m_leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_encoder.setZeroOffset(5.07);

    m_followMotor.follow(m_leadMotor, false);

    m_leadMotor.setIdleMode(IdleMode.kBrake);
    m_followMotor.setIdleMode(IdleMode.kBrake);

    m_encoder.setPositionConversionFactor(PivotConstants.kPivotEncoderPositionFactor);
    m_encoder.setVelocityConversionFactor(PivotConstants.kPivotEncoderVelocityFactor);

    m_leadMotor.burnFlash();
    m_followMotor.burnFlash();

    m_leadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, PivotConstants.kLeaderStatus0PeriodMs);
    m_leadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, PivotConstants.kStatus3PeriodMs);
    m_leadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PivotConstants.kStatus4PeriodMs);
    m_leadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PivotConstants.kLeaderStatus5PeriodMs);

    m_followMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, PivotConstants.kFollowerStatus0PeriodMs);
    m_followMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, PivotConstants.kStatus3PeriodMs);
    m_followMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PivotConstants.kStatus4PeriodMs);
    m_followMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, PivotConstants.kFollowerStatus5PeriodMs);

  }

  public Command pivotForward() {
    return run(this::speedFwd);
  }

  public Command pivotReverse() {
    return run(this::speedRev);
  }

  public Command pivotStop() {
    return runOnce(this::speedStop);
  }

  public double getRadiansFromHorizontal() {
    return m_encoder.getPosition() - Math.PI / 2;
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
    return Commands.run(
            () -> {
              setAngle(point);
            })
        .until(() -> inRange(point));
  }

  public Command pivotSetpointCommand(double radians) {
    double setpoint = radians;
    return Commands.run(() -> setAngle(setpoint)).until(() -> inRange(setpoint));
  }

  private void setAngle(double radians) {
    m_pidController.setReference(radians, ControlType.kPosition);
  }

  private boolean inRange(double setpoint) {
    double measurement = getMeasurement();
    if (setpoint > measurement - .05 && setpoint < measurement + .05) {
      return true;
    } else {
      return false;
    }
  }

  private double getMeasurement() {
    return m_encoder.getPosition();
  }

  public void speedFwd() {
    m_leadMotor.set(.1);
  }

  public void speedRev() {
    m_leadMotor.set(-.1);
  }

  public void speedStop() {
    m_leadMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}