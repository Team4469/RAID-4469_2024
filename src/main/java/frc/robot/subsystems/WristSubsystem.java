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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.WristConstants;
import frc.robot.SetPoints.WristSetpoints;
import java.util.Map;

public class WristSubsystem extends SubsystemBase {
  private final CANSparkFlex m_wristMotor =
      new CANSparkFlex(WristConstants.kWristMotorID, MotorType.kBrushless);

  private final SparkPIDController m_wristPIDController = m_wristMotor.getPIDController();

  private final AbsoluteEncoder m_encoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 2.98, 0.02);

  public final Trigger m_tempTrigger = new Trigger(() -> (m_wristMotor.getMotorTemperature() > 65));

  GenericEntry bolWristTempEntry =
      Shuffleboard.getTab("Wrist")
          .add("WristMotorTemp", false)
          .withWidget("Boolean Box")
          .withProperties(Map.of("colorWhenTrue", "maroon", "colorWhenFalse", "green"))
          .getEntry();

  /** Creates a new WristIntake. */
  public WristSubsystem() {
    m_wristMotor.restoreFactoryDefaults();

    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setInverted(true);
    m_wristMotor.setSmartCurrentLimit(WristConstants.kSmartCurrentLimit);

    m_wristMotor.setClosedLoopRampRate(2);

    // m_wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) WristConstants.kMaxRads);
    m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) WristConstants.kMinRads);

    m_wristMotor.burnFlash();

    m_encoder.setPositionConversionFactor(2.0 * Math.PI);
    m_encoder.setVelocityConversionFactor((2.0 * Math.PI) / 60.0);

    m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, WristConstants.kStatus3PeriodMs);
    m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, WristConstants.kStatus4PeriodMs);
    m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, WristConstants.kStatus5PeriodMs);

    m_wristPIDController.setFeedbackDevice(m_encoder);
    m_wristPIDController.setPositionPIDWrappingEnabled(false);
    m_wristPIDController.setPositionPIDWrappingMaxInput(WristConstants.kMaxRads);
    m_wristPIDController.setPositionPIDWrappingMinInput(WristConstants.kMinRads);
    m_wristPIDController.setP(0.000001, 0);
    m_wristPIDController.setI(0, 0);
    m_wristPIDController.setD(0, 0);
  }

  public Command wristForward() {
    return runOnce(() -> wristSpeed(1));
  }

  public Command wristReverse() {
    return runOnce(() -> wristSpeed(-.1));
  }

  public Command wristStop() {
    return runOnce(() -> wristSpeed(0));
  }

  private void wristSpeed(double Speed) {
    m_wristMotor.set(Speed);
  }

  public Command wristAngleSetpoint(double radians) {
    double setpoint = radians;
    return Commands.run(() -> setAngle(setpoint))
        .until(() -> inRange(setpoint)); // .until(() -> inRange(setpoint))
  }

  private boolean inRange(double setpoint) {
    double measurement = getMeasurement();
    if (setpoint > measurement - Units.inchesToMeters(.25)
        && setpoint < measurement + Units.inchesToMeters(.25)) {
      return true;
    } else {
      return false;
    }
  }

  private double getMeasurement() {
    return m_encoder.getPosition();
  }

  private void setAngle(double radians) {
    m_wristPIDController.setReference(radians, ControlType.kPosition);
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
    return Commands.run(() -> setAngle(point)).until(() -> inRange(point));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    bolWristTempEntry.setBoolean(m_tempTrigger.getAsBoolean());
  }
}
