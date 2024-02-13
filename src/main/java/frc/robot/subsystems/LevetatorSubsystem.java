// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.LevetatorConstants;
import frc.robot.SetPoints.LevetatorSetpoints;

public class LevetatorSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor;

  private final RelativeEncoder m_encoder;

  private final SparkPIDController m_pidController;

  private final double kGravity;

  // private final ElevatorFeedforward m_elevatorFeedforward =
  //     new ElevatorFeedforward(LevetatorConstants.kS, LevetatorConstants.kG,
  // LevetatorConstants.kV);

  private final PivotSubsystem m_pivot;

  /** Creates a new LevSub. */
  public LevetatorSubsystem(PivotSubsystem pivot) {
    m_motor = new CANSparkMax(LevetatorConstants.kLevetatorMotorID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    m_pivot = pivot;

    m_motor.setClosedLoopRampRate(3);

    m_encoder = m_motor.getEncoder();

    m_encoder.setPositionConversionFactor(LevetatorConstants.kPositionConversionFactor);
    m_motor.setSmartCurrentLimit(LevetatorConstants.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) LevetatorConstants.kForwardSoftLimit);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) LevetatorConstants.kReverseSoftLimit);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_motor.setOpenLoopRampRate(2);

    m_pidController = m_motor.getPIDController();

    m_pidController.setFeedbackDevice(m_encoder);

    m_pidController.setPositionPIDWrappingEnabled(false);
    m_pidController.setPositionPIDWrappingMaxInput(Units.inchesToMeters(9));
    m_pidController.setPositionPIDWrappingMinInput(Units.inchesToMeters(0));

    m_pidController.setP(3.2);
    m_pidController.setI(0);
    m_pidController.setD(2);

    m_pidController.setOutputRange(-.1, 1);

    m_motor.setInverted(true);

    m_motor.burnFlash();

    kGravity = 1.6;
  }

  public Command levetatorAmpSmartCommand(AmpDirection ampDirection) {
    double point;
    switch (ampDirection) {
      case FRONT:
        point = LevetatorSetpoints.kAmpFront;
        break;
      default:
        point = LevetatorSetpoints.kAmpRear;
        break;
    }
    return Commands.run(() -> setDistance(point));
  }

  public Command levTest1() {
    double setpoint = Units.inchesToMeters(2);
    return Commands.run(() -> setDistance(setpoint)); // .until(() -> inRange(setpoint))
  }

  public Command levTest2() {
    double setpoint = Units.inchesToMeters(6);
    return Commands.run(() -> setDistance(setpoint)); // .until(() -> inRange(setpoint))
  }

  public Command levetatorSetpointPosition(double meters) {
    double setpoint = meters;
    return Commands.run(() -> setDistance(setpoint))
        .until(() -> inRange(setpoint))
        .finallyDo(this::levHold); // .until(() -> inRange(setpoint))
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

  private void levHold() {
    m_motor.setVoltage(kGravity * Math.sin(m_pivot.getRadiansFromHorizontal()));
  }

  private void setDistance(double meters) {
    m_pidController.setReference(
        meters,
        ControlType.kPosition,
        0,
        kGravity * Math.sin(m_pivot.getRadiansFromHorizontal()),
        ArbFFUnits.kVoltage);
  }

  public Command levForward() {
    return run(this::speedFwd);
  }

  public Command levReverse() {
    return run(this::speedRev);
  }

  public Command levStop() {
    return runOnce(this::speedStop);
  }

  public void speedFwd() {
    m_motor.set(.3);
  }

  public void speedRev() {
    m_motor.set(-.1);
  }

  public void speedStop() {
    m_motor.set(0);
  }

  private double getMeasurement() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
