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
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.LevetatorConstants;
import frc.robot.SetPoints.LevetatorSetpoints;

public class LevetatorSubsystem extends ProfiledPIDSubsystem {

  private final CANSparkMax m_levetatorMotor;

  private final LaserCan m_distanceLaserCan;

  private final ElevatorFeedforward m_elevatorFeedforward =
      new ElevatorFeedforward(LevetatorConstants.kS, LevetatorConstants.kG, LevetatorConstants.kV);

  private final PivotSubsystem m_pivot;

  /** Creates a new LevetatorSubsystem. */
  public LevetatorSubsystem(PivotSubsystem pivot) {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            LevetatorConstants.kP,
            LevetatorConstants.kI,
            LevetatorConstants.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                LevetatorConstants.kMaxVelocity, LevetatorConstants.kMaxAcceleration)));

    m_pivot = pivot;

    m_levetatorMotor = new CANSparkMax(LevetatorConstants.kLevetatorMotorID, MotorType.kBrushless);

    m_distanceLaserCan = new LaserCan(LevetatorConstants.kLevetatorLaserCanID);

    m_levetatorMotor.restoreFactoryDefaults();

    m_levetatorMotor.setIdleMode(IdleMode.kBrake);
    m_levetatorMotor.setSmartCurrentLimit(85);
    m_levetatorMotor.burnFlash();
    
    try {
    m_distanceLaserCan.setRangingMode(RangingMode.SHORT);
    m_distanceLaserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
    m_distanceLaserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);}
    catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    setGoal(m_distanceLaserCan.getMeasurement().distance_mm / 1000.0); // meters
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = m_elevatorFeedforward.calculate(setpoint.position, setpoint.velocity)*Math.sin(m_pivot.getRadiansFromHorizontal());
    // Add the feedforward to the PID output to get the motor output
    m_levetatorMotor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    LaserCan.Measurement distance = m_distanceLaserCan.getMeasurement();
    return (distance.distance_mm * 1000); // position is in meters
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
    return Commands.runOnce(
        () -> {
          this.setGoal(point);
          this.enable();
        },
        this);
  }

  public Command positionMovement() {
    return Commands.runOnce(
        () -> {
          this.setGoal(LevetatorSetpoints.kMovement);
          this.enable();
        },
        this);
  }

  public Command positionIntake() {
    return Commands.runOnce(
        () -> {
          this.setGoal(LevetatorSetpoints.kIntake);
          this.enable();
        },
        this);
  }

  public Command positionStowed() {
    return Commands.runOnce(
        () -> {
          this.setGoal(LevetatorSetpoints.kStowed);
          this.enable();
        },
        this);
  }

  public Command positionSubwoofer() {
    return Commands.runOnce(
        () -> {
          this.setGoal(LevetatorSetpoints.kSubwoofer);
          this.enable();
        },
        this);
  }

  public Command positionAmpFront() {
    return Commands.runOnce(
        () -> {
          this.setGoal(LevetatorSetpoints.kAmpFront);
          this.enable();
        },
        this);
  }

  public Command positionAmpRear() {
    return Commands.runOnce(
        () -> {
          this.setGoal(LevetatorSetpoints.kAmpRear);
          this.enable();
        },
        this);
  }

  public Command positionTrap() {
    return Commands.runOnce(
        () -> {
          this.setGoal(LevetatorSetpoints.kTrap);
          this.enable();
        },
        this);
  }
}
