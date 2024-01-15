// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.LevetatorConstants;
import frc.utils.TunableNumber;

public class LevetatorSubsystem extends ProfiledPIDSubsystem {

  TunableNumber kPTunableNumber = new TunableNumber("levetator/kP", LevetatorConstants.kP);

  private final CANSparkMax m_levetatorMotor =
      new CANSparkMax(LevetatorConstants.kLevetatorMotorID, MotorType.kBrushless);

  private final LaserCan m_distanceLaserCan = new LaserCan(LevetatorConstants.kLevetatorLaserCanID);

  private final ElevatorFeedforward m_elevatorFeedforward =
      new ElevatorFeedforward(LevetatorConstants.kS, LevetatorConstants.kG, LevetatorConstants.kV);

  /** Creates a new LevetatorSubsystem. */
  public LevetatorSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            LevetatorConstants.kP,
            LevetatorConstants.kI,
            LevetatorConstants.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                LevetatorConstants.kMaxVelocity, LevetatorConstants.kMaxAcceleration)));

    m_levetatorMotor.restoreFactoryDefaults();

    m_levetatorMotor.setIdleMode(IdleMode.kBrake);
    m_levetatorMotor.setSmartCurrentLimit(65);
    m_levetatorMotor.burnFlash();

    m_distanceLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
    m_distanceLaserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
    m_distanceLaserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);

    setGoal(0); // goal should be in millimeters!!!
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = m_elevatorFeedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_levetatorMotor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    LaserCan.Measurement distance = m_distanceLaserCan.getMeasurement();
    return distance.distance_mm;
  }
}
