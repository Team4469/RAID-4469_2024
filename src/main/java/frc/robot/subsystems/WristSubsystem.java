// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.utils.ShootingInterpolationTables.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.WristConstants;
import frc.robot.SetPoints.WristSetpoints;
import frc.utils.ShootingInterpolationTables.ShooterLaunchAngleTable;
import java.util.Map;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class WristSubsystem extends SubsystemBase implements Logged {
  private final CANSparkFlex m_wristMotor =
      new CANSparkFlex(WristConstants.kWristMotorID, MotorType.kBrushless);

  private final SparkPIDController m_wristPIDController = m_wristMotor.getPIDController();

  private final AbsoluteEncoder m_encoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private PivotSubsystem m_pivot;

  private final double perpendicularToArmRads = Math.PI;

  private double kGravity = .45;

  private double SETPOINT;
  private boolean SETPOINT_INIT;
  private int init_loop_number = 15;
  private int init_loop_count = 0;
  MedianFilter Setpoint_init_filter = new MedianFilter(init_loop_number);

  public final Trigger m_tempTrigger = new Trigger(() -> (m_wristMotor.getMotorTemperature() > 65));

  GenericEntry bolWristTempEntry =
      Shuffleboard.getTab("Wrist")
          .add("WristMotorTemp", false)
          .withWidget("Boolean Box")
          .withProperties(Map.of("colorWhenTrue", "maroon", "colorWhenFalse", "green"))
          .getEntry();

  /** Creates a new WristIntake. */
  public WristSubsystem(PivotSubsystem pivot) {
    this.m_pivot = pivot;
    m_wristMotor.restoreFactoryDefaults();

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristMotor.getInverted() != true) {
        m_wristMotor.setInverted(true);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristMotor.getIdleMode() != IdleMode.kBrake) {
        m_wristMotor.setIdleMode(IdleMode.kBrake);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    m_wristMotor.setSmartCurrentLimit(115);

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristMotor.getClosedLoopRampRate() != 2) {
        m_wristMotor.setClosedLoopRampRate(2);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristMotor.getSoftLimit(SoftLimitDirection.kForward)
    //       != (float) WristConstants.kMaxRads) {
        m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) WristConstants.kMaxRads);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristMotor.getSoftLimit(SoftLimitDirection.kReverse)
    //       != (float) WristConstants.kMinRads) {
        m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) WristConstants.kMinRads);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_encoder.getPositionConversionFactor() != WristConstants.kPositionConversionFactor) {
        m_encoder.setPositionConversionFactor(WristConstants.kPositionConversionFactor);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_encoder.getVelocityConversionFactor() != WristConstants.kVelocityConversionFactor) {
        m_encoder.setVelocityConversionFactor(WristConstants.kVelocityConversionFactor);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_encoder.getZeroOffset() != WristConstants.kEncoderZeroOffset) {
        m_encoder.setZeroOffset(WristConstants.kEncoderZeroOffset);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_encoder.getInverted() != WristConstants.kEncoderInverted) {
        m_encoder.setInverted(WristConstants.kEncoderInverted);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    m_wristPIDController.setFeedbackDevice(m_encoder);
    m_wristPIDController.setPositionPIDWrappingEnabled(false);
    m_wristPIDController.setPositionPIDWrappingMaxInput(WristConstants.kMaxRads);
    m_wristPIDController.setPositionPIDWrappingMinInput(WristConstants.kMinRads);
    m_wristPIDController.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristPIDController.getIMaxAccum(0) != WristConstants.kIAcum) {
        m_wristPIDController.setIMaxAccum(WristConstants.kIAcum, 0);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristPIDController.getIZone() != WristConstants.kIz) {
        m_wristPIDController.setIZone(WristConstants.kIz);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristPIDController.getI() != WristConstants.kI) {
        m_wristPIDController.setI(WristConstants.kI);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristPIDController.getP() != WristConstants.kP) {
        m_wristPIDController.setP(WristConstants.kP);
    //   } else {
    //     System.out.println("Set Wrist P to : " + WristConstants.kP);
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_wristPIDController.getD() != WristConstants.kD) {
        m_wristPIDController.setD(WristConstants.kD);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    m_wristMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus0, 10); // Output, Faults, Sticky Faults, Is Follower
    m_wristMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus1, 20); // Motor Velo, Motor Temp, Motor Volts, Motor Current
    m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // Motor Position
    m_wristMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus3,
        500); // Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_wristMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus4, 500); // Alternate Encoder Velocity, Alternate Encoder Position
    m_wristMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus5, 100); // Absolute Encoder Position, Absolute Encoder Angle
    m_wristMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus6, 100); // Absolute Encoder Velocity, Absolute Encoder Frequency

    m_wristMotor.burnFlash();

    SETPOINT_INIT = false;
  }

  public Command wristAngleVariableSetpoint(DoubleSupplier distanceToTarget) {
    double setpoint =
        ShooterLaunchAngleTable.SHOOTER_LAUNCH_ANGLE_INTERP_TABLE.get(
            distanceToTarget.getAsDouble());
    SmartDashboard.putNumber("Wrist Angle", setpoint);
    return Commands.runOnce(() -> setAngle(setpoint));
  }

  public Command wristAngleSetpoint(double radians) {
    double setpoint = radians;
    return Commands.runOnce(() -> setSetpoint(setpoint)); // .until(() -> inRange(setpoint))
  }

  public Command wristInRange() {
    return Commands.waitUntil(() -> inRange(getSetpoint()));
  }

  public boolean inRange(double setpoint) {
    double measurement = getMeasurement();
    if (setpoint > measurement - Units.degreesToRadians(1)
        && setpoint < measurement + Units.degreesToRadians(1)) {
      return true;
    } else {
      return false;
    }
  }

  @Log
  private double getMeasurement() {
    return m_encoder.getPosition();
  }

  @Log
  private boolean getCanRxFault() {
    return m_wristMotor.getFault(FaultID.kCANRX);
  }

  @Log
  private boolean getCanTxFault() {
    return m_wristMotor.getFault(FaultID.kCANTX);
  }

  private void setAngle(double radians) {
    m_wristPIDController.setReference(radians, ControlType.kPosition);
  }

  public Command wristAmpSmartCommand(AmpDirection ampSelect) {
    var amp = ampSelect;
    SmartDashboard.putString("Wrist Amp Dir", "" + amp);
    double point;
    if (amp == AmpDirection.FRONT) {
      point = WristSetpoints.kAmpFront;
    } else {
      point = WristSetpoints.kAmpRear;
    }
    return Commands.runOnce(() -> setSetpoint(point));
  }

  @Log
  private double getI() {
    return m_wristPIDController.getIAccum();
  }

  public void setSetpoint(double radians) {
    SETPOINT = radians;
    m_wristPIDController.setIAccum(0);
    m_wristPIDController.setReference(SETPOINT, ControlType.kPosition);
  }

  private double wristRadsFromHorizontal() {
    var pivAngle = m_pivot.getRadiansFromHorizontal();
    var wristAngle = wristRadsFromArmHorizontal();
    return (pivAngle - Math.PI / 2 - wristAngle);
  }

  private double wristRadsFromArmHorizontal() {
    return getMeasurement() - perpendicularToArmRads;
  }

  @Log
  private double getSetpoint() {
    return SETPOINT;
  }

  @Log
  private double getOutput() {
    return m_wristMotor.getAppliedOutput();
  }

  @Log
  private boolean getSetpointInit() {
    return SETPOINT_INIT;
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

    m_wristPIDController.setReference(
        getSetpoint(),
        ControlType.kPosition,
        0,
        kGravity * Math.cos(wristRadsFromHorizontal()),
        ArbFFUnits.kVoltage);

    bolWristTempEntry.setBoolean(m_tempTrigger.getAsBoolean());
    SmartDashboard.putNumber("Wrist Setpoint", getSetpoint());
    SmartDashboard.putNumber("Wrist Encoder", m_encoder.getPosition());
    SmartDashboard.putNumber("Wrist Current", m_wristMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Wrist In Range", inRange(getSetpoint()));
    SmartDashboard.putNumber("Wrist Deg From Horz", wristRadsFromHorizontal());
  }
}
