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

public class WristSubsystem extends SubsystemBase{
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

    m_encoder.setZeroOffset(WristConstants.kEncoderZeroOffset);
    m_encoder.setInverted(WristConstants.kEncoderInverted);

    m_wristPIDController.setFeedbackDevice(m_encoder);
    m_wristPIDController.setPositionPIDWrappingEnabled(false);
    m_wristPIDController.setPositionPIDWrappingMaxInput(WristConstants.kMaxRads);
    m_wristPIDController.setPositionPIDWrappingMinInput(WristConstants.kMinRads);
    m_wristPIDController.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
    m_wristPIDController.setP(WristConstants.kP);
    m_wristPIDController.setI(WristConstants.kI);
    m_wristPIDController.setD(WristConstants.kD);
    m_wristPIDController.setIMaxAccum(WristConstants.kIAcum, 0);
    m_wristPIDController.setIZone(WristConstants.kIz);

    for (int i = 0; i < 6; i++) {
      if (m_wristPIDController.getI() != WristConstants.kI) {
        m_wristPIDController.setI(WristConstants.kI);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_wristPIDController.getP() != WristConstants.kP) {
        m_wristPIDController.setP(WristConstants.kP);
      } else {
        System.out.println("Set Wrist P to : " + WristConstants.kP);
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_wristPIDController.getD() != WristConstants.kD) {
        m_wristPIDController.setD(WristConstants.kD);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    m_wristMotor.burnFlash();

    // m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, WristConstants.kStatus3PeriodMs);
    // m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, WristConstants.kStatus4PeriodMs);
    // m_wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, WristConstants.kStatus5PeriodMs);
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
    if (setpoint > measurement - Units.degreesToRadians(5)
        && setpoint < measurement + Units.degreesToRadians(5)) {
      return true;
    } else {
      return false;
    }
  }

  @Log
  private double getMeasurement() {
    return m_encoder.getPosition();
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
