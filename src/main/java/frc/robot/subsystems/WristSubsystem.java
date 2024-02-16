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
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private double SETPOINT;
  private boolean SETPOINT_INIT;
  private int init_loop_number = 15;
  private int init_loop_count = 0;
  MedianFilter Setpoint_init_filter = new MedianFilter(init_loop_number);

  private double kGravity;
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
    m_wristMotor.setSmartCurrentLimit(115);

    m_wristMotor.setClosedLoopRampRate(2);

    // m_wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) WristConstants.kMaxRads);
    m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) WristConstants.kMinRads);

    m_encoder.setPositionConversionFactor(WristConstants.kPositionConversionFactor);
    m_encoder.setVelocityConversionFactor(WristConstants.kVelocityConversionFactor);

    m_encoder.setZeroOffset(2.688);
    m_encoder.setInverted(false);

    m_wristPIDController.setFeedbackDevice(m_encoder);
    m_wristPIDController.setPositionPIDWrappingEnabled(false);
    m_wristPIDController.setPositionPIDWrappingMaxInput(WristConstants.kMaxRads);
    m_wristPIDController.setPositionPIDWrappingMinInput(WristConstants.kMinRads);
    m_wristPIDController.setOutputRange(-1, 1);
    m_wristPIDController.setP(.65);
    m_wristPIDController.setI(0);
    m_wristPIDController.setD(10);

    m_wristMotor.burnFlash();

    // m_encoder.setPositionConversionFactor(2.0 * Math.PI);
    // m_encoder.setVelocityConversionFactor((2.0 * Math.PI) / 60.0);
    kGravity = .5;

    // m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, WristConstants.kStatus3PeriodMs);
    // m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, WristConstants.kStatus4PeriodMs);
    // m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, WristConstants.kStatus5PeriodMs);
    SETPOINT_INIT = false;
    // setSetpoint(2.5);
  }

  // public Command wristForward() {
  //   return runOnce(() -> wristSpeed(.1));
  // }

  // public Command wristReverse() {
  //   return runOnce(() -> wristSpeed(-.1));
  // }

  // public Command wristStop() {
  //   return runOnce(() -> wristSpeed(0));
  // }

  // private void wristSpeed(double Speed) {
  //   m_wristMotor.set(Speed);
  // }

  public Command wristAngleSetpoint(double radians) {
    double setpoint = radians;
    return Commands.runOnce(() -> setSetpoint(setpoint)); // .until(() -> inRange(setpoint))
  }

  public Command wristInRange() {
    return Commands.waitUntil(() -> inRange(getSetpoint()));
  }

  // public Command wristTest1() {
  //   return Commands.run(() -> setAngle(2.6))
  //       .until(() -> inRange(2.6)); // .until(() -> inRange(setpoint))
  // }

  //   public Command wristTest2() {
  //     return Commands.run(() -> setAngle(3.2))
  //         .until(() -> inRange(3.2)); // .until(() -> inRange(setpoint))
  //   }

  public boolean inRange(double setpoint) {
    double measurement = getMeasurement();
    if (setpoint > measurement - Units.degreesToRadians(2)
        && setpoint < measurement + Units.degreesToRadians(2)) {
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

  private void setSetpoint(double radians) {
    SETPOINT = radians;
    m_wristPIDController.setReference(SETPOINT, ControlType.kPosition);
  }

  private double getSetpoint() {
    return SETPOINT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!SETPOINT_INIT) {
      setSetpoint(Setpoint_init_filter.calculate(getMeasurement()));
      if (init_loop_count <= init_loop_number) {
        init_loop_count++;
      } else {
        SETPOINT_INIT = true;
      }
    }

    bolWristTempEntry.setBoolean(m_tempTrigger.getAsBoolean());
    SmartDashboard.putNumber("Wrist Setpoint", getSetpoint());
    SmartDashboard.putNumber("Wrist Encoder", m_encoder.getPosition());
    SmartDashboard.putNumber("Wrist Current", m_wristMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Wrist In Range", inRange(getSetpoint()));

  }
}
