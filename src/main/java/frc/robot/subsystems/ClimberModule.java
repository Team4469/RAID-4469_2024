// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import au.grapplerobotics.ConfigurationFailedException;
// import au.grapplerobotics.LaserCan;
// import au.grapplerobotics.LaserCan.RangingMode;
// import au.grapplerobotics.LaserCan.RegionOfInterest;
// import au.grapplerobotics.LaserCan.TimingBudget;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberModule extends SubsystemBase {

  private final CANSparkMax m_climbingMotor;
  private final RelativeEncoder m_encoder;
  private final SparkPIDController m_pidController;

  // private LaserCan m_distanceSensor;

  int ID;

  private static final double ALLOWABLE_POSITION_ERROR = Units.inchesToMeters(0.5);

  private Mode mode = Mode.VOLTAGE;
  private boolean zeroed = false;
  private double targetHeight = 0.0;
  private double targetVoltage = 0.0;

  /** Creates a new ClimberModule. */
  public ClimberModule(int MotorCanID, int LaserCanID, boolean MotorInverted) {

    m_climbingMotor = new CANSparkMax(MotorCanID, MotorType.kBrushless);

    m_climbingMotor.restoreFactoryDefaults();

    m_encoder = m_climbingMotor.getEncoder();

    m_encoder.setPositionConversionFactor(
        ClimberConstants.kPositionConversionFactor); // Rotations to meters
    m_encoder.setVelocityConversionFactor(ClimberConstants.kPositionConversionFactor / 60);
    m_climbingMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.inchesToMeters(24));
    m_climbingMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(0));
    m_climbingMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_climbingMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    m_climbingMotor.setIdleMode(IdleMode.kBrake);
    m_climbingMotor.setSmartCurrentLimit(80);
    m_climbingMotor.setInverted(MotorInverted);
    m_climbingMotor.enableVoltageCompensation(12);

    m_pidController = m_climbingMotor.getPIDController();

    m_pidController.setP(0.002, PID_Slot.NO_LOAD.ordinal());
    m_pidController.setI(0, PID_Slot.NO_LOAD.ordinal());
    m_pidController.setD(0, PID_Slot.NO_LOAD.ordinal());
    m_pidController.setFF(0, PID_Slot.NO_LOAD.ordinal());

    m_pidController.setP(.003, PID_Slot.CLIMBING.ordinal());
    m_pidController.setI(0, PID_Slot.CLIMBING.ordinal());
    m_pidController.setD(0, PID_Slot.CLIMBING.ordinal());
    m_pidController.setFF(0, PID_Slot.CLIMBING.ordinal());

    m_climbingMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus0, 10); // Output, Faults, Sticky Faults, Is Follower
    m_climbingMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus1, 20); // Motor Velo, Motor Temp, Motor Volts, Motor Current
    m_climbingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // Motor Position
    m_climbingMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus3,
        500); // Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_climbingMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus4, 500); // Alternate Encoder Velocity, Alternate Encoder Position
    m_climbingMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus5, 500); // Absolute Encoder Position, Absolute Encoder Angle
    m_climbingMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus6, 500); // Absolute Encoder Velocity, Absolute Encoder Frequency

    Timer.delay(.25);

    m_climbingMotor.burnFlash();

    // m_distanceSensor = new LaserCan(LaserCanID);
    // ID = LaserCanID;

    // try {
    //   m_distanceSensor.setRangingMode(RangingMode.SHORT);
    //   m_distanceSensor.setRegionOfInterest(new RegionOfInterest(8, 12, 4, 4));
    //   m_distanceSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
    // } catch (ConfigurationFailedException e) {
    //   System.out.println("Configuration failed! " + e);
    // }

    // // LaserCan.Measurement measure = m_distanceSensor.getMeasurement();
    // // m_encoder.setPosition(measure.distance_mm / 1000);
  }

  public Command setHeight(double meters) {
    return Commands.runOnce(() -> setTargetHeight(meters, false));
  }

  public double getTargetHeight() {
    if (mode == Mode.POSITION_NO_LOAD || mode == Mode.POSITION_CLIMBING) {
      return targetHeight;
    }
    return 0.0;
  }

  public void setTargetHeight(double target, boolean isClimbing) {
    this.targetHeight = MathUtil.clamp(target, 0, 24);
    if (isClimbing) {
      this.mode = Mode.POSITION_CLIMBING;
    } else {
      this.mode = Mode.POSITION_NO_LOAD;
    }
  }

  public void setTargetVoltage(double voltage) {
    m_climbingMotor.setVoltage(voltage);
    this.mode = Mode.VOLTAGE;
  }

  public void setZeroPosition() {
    m_encoder.setPosition(0);
  }

  public void setZeroed(boolean zeroed) {
    this.zeroed = zeroed;
  }

  public boolean isClimberZeroed() {
    return this.zeroed;
  }

  public double getCurrentHeight() {
    return m_encoder.getPosition();
  }

  public double getCurrentVelocity() {
    return m_encoder.getVelocity();
  }

  public boolean isAtTargetPosition() {
    return Math.abs(getCurrentHeight() - getTargetHeight()) < ALLOWABLE_POSITION_ERROR;
  }

  public void setGains(PID_Slot slot, double p, double i, double d, double ff) {
    for (int j = 0; j < 6; j++) {
      if (m_pidController.getP(slot.ordinal()) != p) {
        m_pidController.setP(p, slot.ordinal());
      } else {
        break;
      }
      Timer.delay(.1);
    }
    for (int j = 0; j < 6; j++) {
      if (m_pidController.getI(slot.ordinal()) != i) {
        m_pidController.setI(i, slot.ordinal());
      } else {
        break;
      }
      Timer.delay(.1);
    }
    for (int j = 0; j < 6; j++) {
      if (m_pidController.getD(slot.ordinal()) != d) {
        m_pidController.setD(d, slot.ordinal());
      } else {
        break;
      }
      Timer.delay(.1);
    }
    for (int j = 0; j < 6; j++) {
      if (m_pidController.getFF(slot.ordinal()) != ff) {
        m_pidController.setFF(ff, slot.ordinal());
      } else {
        break;
      }
      Timer.delay(.1);
    }
  }

  public void turnSoftLimitOn() {
    m_climbingMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  @Override
  public void periodic() {
    switch (mode) {
      case POSITION_NO_LOAD:
        if (isClimberZeroed()) {
          m_pidController.setReference(
              targetHeight,
              ControlType.kPosition,
              PID_Slot.NO_LOAD.ordinal(),
              0.1,
              ArbFFUnits.kVoltage);
        }
        break;
      case POSITION_CLIMBING:
        if (isClimberZeroed()) {
          m_pidController.setReference(
              targetHeight,
              ControlType.kPosition,
              PID_Slot.CLIMBING.ordinal(),
              7,
              ArbFFUnits.kVoltage);
        }
        break;
      case VOLTAGE:
        m_climbingMotor.setVoltage(targetVoltage);
        break;
    }

    SmartDashboard.putNumber(
        m_climbingMotor.getDeviceId() + " Output", m_climbingMotor.getAppliedOutput());
    SmartDashboard.putNumber(m_climbingMotor.getDeviceId() + " Position", getCurrentHeight());
    SmartDashboard.putNumber(m_climbingMotor.getDeviceId() + " Setpoint", getTargetHeight());
    SmartDashboard.putNumber(m_climbingMotor.getDeviceId() + " Velocity", getCurrentVelocity());
    SmartDashboard.putNumber(
        m_climbingMotor.getDeviceId() + " Current", m_climbingMotor.getOutputCurrent());
  }

  private enum Mode {
    POSITION_NO_LOAD,
    POSITION_CLIMBING,
    VOLTAGE
  }

  public enum PID_Slot {
    NO_LOAD,
    CLIMBING
  }
}
