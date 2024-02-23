// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberModule extends SubsystemBase {

  private final CANSparkMax m_climbingMotor;
  private final RelativeEncoder m_encoder;

  private LaserCan m_distanceSensor;

  int ID;

  private boolean EncoderSet = false;

  /** Creates a new ClimberModule. */
  public ClimberModule(int MotorCanID, int LaserCanID, boolean MotorInverted) {

    m_climbingMotor = new CANSparkMax(MotorCanID, MotorType.kBrushless);

    m_climbingMotor.restoreFactoryDefaults();

    m_encoder = m_climbingMotor.getEncoder();

    m_encoder.setPositionConversionFactor(
        ClimberConstants.kPositionConversionFactor); // Rotations to meters
    m_encoder.setVelocityConversionFactor(ClimberConstants.kPositionConversionFactor / 60);
    m_climbingMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.inchesToMeters(20));
    m_climbingMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(3));
    m_climbingMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_climbingMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_climbingMotor.setIdleMode(IdleMode.kBrake);
    m_climbingMotor.setSmartCurrentLimit(85);
    m_climbingMotor.setInverted(MotorInverted);
    m_climbingMotor.burnFlash();

    m_distanceSensor = new LaserCan(LaserCanID);
    ID = LaserCanID;

    try {
      m_distanceSensor.setRangingMode(RangingMode.SHORT);
      m_distanceSensor.setRegionOfInterest(new RegionOfInterest(8, 12, 4, 4));
      m_distanceSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    // LaserCan.Measurement measure = m_distanceSensor.getMeasurement();
    // m_encoder.setPosition(measure.distance_mm / 1000);
  }

  // Commands

  /**
   * @param Goal distance in meters to extend from rest
   * @return command that will run until setpoint is hit
   */
  public Command extendClimber(double Goal) {
    return run(this::extendClimber)
        .until(() -> hookAtSetpoint(Goal))
        .finallyDo((interrupted) -> this.stopClimber());
  }

  /**
   * @param Goal distance in meters to extend from rest
   * @return command that will run until setpoint is hit
   */
  public Command retractClimber(double Goal) {
    return run(this::retractClimber)
        .until(() -> hookAtSetpoint(Goal))
        .finallyDo((interrupted) -> this.stopClimber());
  }

  public Command climberForward() {
    System.out.println(m_encoder.getPosition());
    return runOnce(() -> runClimber(.3));
  }

  public Command climberReverse() {
    System.out.println(m_encoder.getPosition());
    return runOnce(() -> runClimber(-.4));
  }

  /**
   * @param Goal distance in meters to extend from rest
   * @return command that will run until setpoint is hit
   */
  public Command climbClimber(double Goal) {
    return run(this::retractClimber)
        .until(() -> hookAtSetpoint(Goal))
        .finallyDo((interrupted) -> this.holdClimber());
  }

  public Command emergencyStopClimberCommand() {
    return runOnce(this::stopClimber);
  }

  private boolean hookAtSetpoint(double setpoint) {
    double currentPos =
        (m_distanceSensor.getMeasurement().distance_mm / 1000.0) + ClimberConstants.kSensorOffset;
    if (currentPos >= setpoint) {
      return true;
    }
    return false;
  }

  private void extendClimber() {
    m_climbingMotor.set(ClimberConstants.extensionSpeed);
  }

  private void retractClimber() {
    m_climbingMotor.set(ClimberConstants.retractionSpeed);
  }

  private void runClimber(double Speed) {
    m_climbingMotor.set(Speed);
  }

  private void holdClimber() {
    m_climbingMotor.setVoltage(1);
  }

  private void stopClimber() {
    m_climbingMotor.set(0);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    LaserCan.Measurement measurement = m_distanceSensor.getMeasurement();
    // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
    // {
    //   System.out.println("LaserCAN " + ID + ": The target is " + measurement.distance_mm + "mm
    // away!");
    // } else {
    //   System.out.println("Oh no! The target is out of range, or we can't get a reliable
    // measurement!");
    //   // You can still use distance_mm in here, if you're ok tolerating a clamped value or an
    // unreliable measurement.
    // }

    // m_encoder.setPosition(Units.inchesToMeters(5));

    if (!EncoderSet) {
      try {
        m_encoder.setPosition((measurement.distance_mm) / 1000.0);
        System.out.println(ID + "encoder set at " + m_encoder.getPosition());
      } catch (Exception e) {
        // System.out.println("Encoder " + ID + " not yet set");
      }
    }

    if (m_encoder.getPosition() != 0) {
      EncoderSet = true;
    }

    // System.out.println(m_encoder.getPosition());
  }
}
