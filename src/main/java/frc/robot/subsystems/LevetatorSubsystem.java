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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.LevetatorConstants;
import frc.robot.SetPoints.LevetatorSetpoints;

public class LevetatorSubsystem extends SubsystemBase {
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

    m_motor.setClosedLoopRampRate(LevetatorConstants.kClosedLoopRampRate);

    m_encoder = m_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    m_encoder.setPositionConversionFactor(LevetatorConstants.kPositionConversionFactor);
    m_motor.setSmartCurrentLimit(LevetatorConstants.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) LevetatorConstants.kForwardSoftLimit);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) LevetatorConstants.kReverseSoftLimit);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_pidController = m_motor.getPIDController();

    m_pidController.setFeedbackDevice(m_encoder);

    m_pidController.setPositionPIDWrappingEnabled(false);
    m_pidController.setPositionPIDWrappingMaxInput(Units.inchesToMeters(9));
    m_pidController.setPositionPIDWrappingMinInput(Units.inchesToMeters(0));

    m_pidController.setP(LevetatorConstants.kP);
    m_pidController.setI(LevetatorConstants.kI);
    m_pidController.setD(LevetatorConstants.kD);

    m_pidController.setOutputRange(LevetatorConstants.kMinOutput, LevetatorConstants.kMaxOutput);

    m_motor.setInverted(LevetatorConstants.kMotorInverted);

    m_motor.burnFlash();

    // m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, LevetatorConstants.kStatus3PeriodMs);
    // m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, LevetatorConstants.kStatus4PeriodMs);
    // m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, LevetatorConstants.kStatus5PeriodMs);

    m_distanceSensor = new LaserCan(LevetatorConstants.kLevetatorLaserCanID);
    ID = LevetatorConstants.kLevetatorLaserCanID;

    try {
      m_distanceSensor.setRangingMode(RangingMode.SHORT);
      m_distanceSensor.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
      m_distanceSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    m_encoder.setPosition(0);
    SETPOINT_INIT = false;
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
    return Commands.runOnce(() -> setSetpoint(setpoint));
    // .finallyDo(this::levHold); // .until(() -> inRange(setpoint))
  }

  public Command levInRange() {
    return Commands.waitUntil(() -> inRange(getSetpoint()));
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

  private void setDistance(double meters) {
    m_pidController.setReference(
        meters,
        ControlType.kPosition,
        0,
        LevetatorConstants.kGravity * Math.sin(m_pivot.getRadiansFromHorizontal()),
        ArbFFUnits.kVoltage);
  }

  public void speedStop() {
    m_motor.set(0);
  }

  private double getMeasurement() {
    return m_encoder.getPosition();
  }

  public void setSetpoint(double radians) {
    SETPOINT = radians;
  }

  private double getSetpoint() {
    return SETPOINT;
  }

  public void resetSetpointInit() {
    SETPOINT_INIT = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LaserCan.Measurement measurement = m_distanceSensor.getMeasurement();

    if (!EncoderSet) {
      try {
        m_encoder.setPosition(
            ((measurement.distance_mm) / 1000.0) + LevetatorConstants.kLevetatorOffset);
        System.out.println(ID + "encoder set at " + m_encoder.getPosition());
        EncoderSet = true;
      } catch (Exception e) {
        // System.out.println("Encoder " + ID + " not yet set");
        EncoderSet = false;
      }
    }

    if (!SETPOINT_INIT && EncoderSet) {
      setSetpoint(Setpoint_init_filter.calculate(getMeasurement()));
      if (init_loop_count <= init_loop_number) {
        init_loop_count++;
      } else {
        SETPOINT_INIT = true;
      }
    }

    // if (m_encoder.getPosition() != 0) {
    //   EncoderSet = true;
    // }

    // System.out.println(m_encoder.getPosition());

    m_pidController.setReference(
        getSetpoint(),
        ControlType.kPosition,
        0,
        LevetatorConstants.kGravity * Math.sin(m_pivot.getRadiansFromHorizontal()),
        ArbFFUnits.kVoltage);

    // SmartDashboard.putNumber("Levetator LaserCAN", measurement.distance_mm);
    SmartDashboard.putNumber("Levetator Setpoint", getSetpoint());
    SmartDashboard.putNumber("Levtator Encoder", m_encoder.getPosition());
    SmartDashboard.putBoolean("Levetator In Range", inRange(getSetpoint()));
  }
}
