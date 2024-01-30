// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberModule extends SubsystemBase {

  private final CANSparkMax m_climbingMotor;

  private final LaserCan m_distanceSensor;

  /** Creates a new ClimberModule. */
  public ClimberModule(int MotorCanID, int LaserCanID, boolean MotorInverted) {

    m_climbingMotor = new CANSparkMax(MotorCanID, MotorType.kBrushless);

    m_climbingMotor.restoreFactoryDefaults();

    m_climbingMotor.setIdleMode(IdleMode.kBrake);
    m_climbingMotor.setSmartCurrentLimit(85);
    m_climbingMotor.burnFlash();
    m_climbingMotor.setInverted(MotorInverted);

    m_distanceSensor = new LaserCan(LaserCanID);

    m_distanceSensor.setRangingMode(RangingMode.SHORT);
    m_distanceSensor.setRegionOfInterest(new RegionOfInterest(8, 8, 2, 2));
    m_distanceSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
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

  private void holdClimber() {
    m_climbingMotor.setVoltage(1);
  }

  private void stopClimber() {
    m_climbingMotor.set(0);
  }

  @Override
  public void periodic() {}
}
