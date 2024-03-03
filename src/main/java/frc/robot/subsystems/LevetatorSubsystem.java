// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.LevetatorConstants;
import frc.robot.SetPoints.LevetatorSetpoints;
import monologue.Annotations.Log;
import monologue.Logged;

public class LevetatorSubsystem extends SubsystemBase implements Logged {
  private double SETPOINT;
  private final CANSparkMax m_motor;

  private final RelativeEncoder m_encoder;

  private final SparkPIDController m_pidController;

  private LaserCan m_distanceSensor;

  int ID;

  private boolean EncoderSet = false;

  private boolean SETPOINT_INIT;
  private int init_loop_number = 15;
  private int init_loop_count = 0;
  MedianFilter Setpoint_init_filter = new MedianFilter(init_loop_number);

  // private final ElevatorFeedforward m_elevatorFeedforward =
  //     new ElevatorFeedforward(LevetatorConstants.kS, LevetatorConstants.kG,
  // LevetatorConstants.kV);

  private final PivotSubsystem m_pivot;

  /** Creates a new LevSub. */
  public LevetatorSubsystem(PivotSubsystem pivot) {
    m_motor = new CANSparkMax(LevetatorConstants.kLevetatorMotorID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    m_pivot = pivot;

    for (int i = 0; i < 6; i++) {
      if (m_motor.getClosedLoopRampRate() != LevetatorConstants.kClosedLoopRampRate) {
        m_motor.setClosedLoopRampRate(LevetatorConstants.kClosedLoopRampRate);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    m_encoder = m_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    for (int i = 0; i < 6; i++) {
      if (m_encoder.getPositionConversionFactor() != LevetatorConstants.kPositionConversionFactor) {
        m_encoder.setPositionConversionFactor(LevetatorConstants.kPositionConversionFactor);
      } else {
        break;
      }
      Timer.delay(.1);
    }
    m_motor.setSmartCurrentLimit(LevetatorConstants.kCurrentLimit);

    for (int i = 0; i < 6; i++) {
      if (m_motor.getIdleMode() != IdleMode.kBrake) {
        m_motor.setIdleMode(IdleMode.kBrake);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_motor.getSoftLimit(SoftLimitDirection.kForward)
          != (float) LevetatorConstants.kForwardSoftLimit) {
        m_motor.setSoftLimit(
            SoftLimitDirection.kForward, (float) LevetatorConstants.kForwardSoftLimit);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_motor.getSoftLimit(SoftLimitDirection.kReverse)
          != (float) LevetatorConstants.kReverseSoftLimit) {
        m_motor.setSoftLimit(
            SoftLimitDirection.kReverse, (float) LevetatorConstants.kForwardSoftLimit);
      } else {
        break;
      }
      Timer.delay(.1);
    }
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_pidController = m_motor.getPIDController();

    m_pidController.setFeedbackDevice(m_encoder);

    m_pidController.setPositionPIDWrappingEnabled(false);
    m_pidController.setPositionPIDWrappingMaxInput(Units.inchesToMeters(9));
    m_pidController.setPositionPIDWrappingMinInput(Units.inchesToMeters(0));

    for (int i = 0; i < 6; i++) {
      if (m_pidController.getI() != LevetatorConstants.kI) {
        m_pidController.setI(LevetatorConstants.kI);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_pidController.getP() != LevetatorConstants.kP) {
        m_pidController.setP(LevetatorConstants.kP);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_pidController.getD() != LevetatorConstants.kD) {
        m_pidController.setD(LevetatorConstants.kD);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    m_pidController.setOutputRange(LevetatorConstants.kMinOutput, LevetatorConstants.kMaxOutput);

    for (int i = 0; i < 6; i++) {
      if (m_motor.getInverted() != LevetatorConstants.kMotorInverted) {
        m_motor.setInverted(LevetatorConstants.kMotorInverted);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    m_motor.burnFlash();

    m_motor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus0, 10); // Output, Faults, Sticky Faults, Is Follower
    m_motor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus1, 20); // Motor Velo, Motor Temp, Motor Volts, Motor Current
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // Motor Position
    m_motor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus3,
        500); // Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_motor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus4, 500); // Alternate Encoder Velocity, Alternate Encoder Position
    m_motor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus5, 500); // Absolute Encoder Position, Absolute Encoder Angle
    m_motor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus6, 500); // Absolute Encoder Velocity, Absolute Encoder Frequency

    m_distanceSensor = new LaserCan(LevetatorConstants.kLevetatorLaserCanID);
    ID = LevetatorConstants.kLevetatorLaserCanID;

    try {
      m_distanceSensor.setRangingMode(RangingMode.SHORT);
      m_distanceSensor.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
      m_distanceSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    m_encoder.setPosition(0.0);
    SETPOINT_INIT = true;
  }

  public Command levetatorAmpSmartCommand(AmpDirection ampSelect) {

    var amp = ampSelect;
    SmartDashboard.putString("Lev Amp Dir", "" + amp);
    double point;
    if (amp == AmpDirection.FRONT) {
      point = LevetatorSetpoints.kAmpFront;
    } else {
      point = LevetatorSetpoints.kAmpRear;
    }
    return Commands.runOnce(() -> setSetpoint(point));
  }

  public Command levetatorSetpointPosition(double meters) {
    double setpoint = meters;
    this.setFlingyMode();
    return Commands.runOnce(() -> setSetpoint(setpoint));
    // .finallyDo(this::levHold); // .until(() -> inRange(setpoint))
  }

  public Command levInRange() {
    return Commands.waitUntil(() -> inRange(getSetpoint()));
  }

  public Command setSquishyModeCommand() {
    return runOnce(this::setSquishyMode);
  }

  public boolean inRange(double setpoint) {
    double measurement = getMeasurement();
    if (setpoint > measurement - Units.inchesToMeters(.25)
        && setpoint < measurement + Units.inchesToMeters(.25)) {
      return true;
    } else {
      return false;
    }
  }

  public void speedStop() {
    m_motor.set(0);
  }

  @Log
  private double getMeasurement() {
    return m_encoder.getPosition();
  }

  public void setSetpoint(double meters) {
    SETPOINT = meters;
  }

  @Log
  private double getSetpoint() {
    return SETPOINT;
  }

  public void resetSetpointInit() {
    SETPOINT_INIT = false;
  }

  @Log
  public double getAppliedOutput() {
    return m_motor.getAppliedOutput();
  }

  @Log
  public double getCurrent() {
    return m_motor.getOutputCurrent();
  }

  public void setSquishyMode() {
    m_motor.setSmartCurrentLimit(5);
  }

  public void setFlingyMode() {
    m_motor.setSmartCurrentLimit(LevetatorConstants.kCurrentLimit);
  }

  // @Log
  // public double getLaserMeasurement(){
  //   return m_motor.getOutputCurrent();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LaserCan.Measurement measurement = m_distanceSensor.getMeasurement();
    SmartDashboard.putNumber("Levetator Laset", measurement.distance_mm / 1000.0);

    // if (!EncoderSet) {
    //   try {
    //     m_encoder.setPosition(
    //         ((measurement.distance_mm) / 1000.0) + LevetatorConstants.kLevetatorOffset);
    //     System.out.println(ID + "encoder set at " + m_encoder.getPosition());
    //     EncoderSet = true;
    //   } catch (Exception e) {
    //     // System.out.println("Encoder " + ID + " not yet set");
    //     EncoderSet = false;
    //   }
    // }

    // if (!SETPOINT_INIT && EncoderSet) {
    //   setSetpoint(Setpoint_init_filter.calculate(getMeasurement()));
    //   if (init_loop_count <= init_loop_number) {
    //     init_loop_count++;
    //   } else {
    //     SETPOINT_INIT = true;
    //   }
    // }

    // if (m_encoder.getPosition() != 0) {
    //   EncoderSet = true;
    // }

    // System.out.println(m_encoder.getPosition());

    m_pidController.setReference(getSetpoint(), ControlType.kPosition);

    // m_pidController.setReference(
    //     getSetpoint(),
    //     ControlType.kPosition,
    //     0,
    //     LevetatorConstants.kGravity * Math.sin(m_pivot.getRadiansFromHorizontal()),
    //     ArbFFUnits.kVoltage);

    // SmartDashboard.putNumber("Levetator LaserCAN", measurement.distance_mm);
    SmartDashboard.putNumber("Levetator Setpoint", getSetpoint());
    SmartDashboard.putNumber("Levtator Encoder", m_encoder.getPosition());
    SmartDashboard.putBoolean("Levetator In Range", inRange(getSetpoint()));
  }
}
