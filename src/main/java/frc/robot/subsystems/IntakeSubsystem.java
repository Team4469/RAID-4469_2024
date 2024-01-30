// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GlobalSensors;
import frc.robot.Constants.IntakeConstants;
import frc.utils.TunableNumber;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkFlex m_intakeMotor;

  TunableNumber INTAKE_SPEED = new TunableNumber("Intake/Intake speed", .5);
  TunableNumber OUTTAKE_SPEED =
      new TunableNumber("Intake/Outtake speed", -.5); // FOR AMP FROM FRONT
  TunableNumber TRANSFER_FORWARD_SPEED = new TunableNumber("Intake/Transfer fwd speed", .1);
  TunableNumber TRANSFER_BACKWARD_SPEED = new TunableNumber("Intake/Transfer bck speed", -.1);

  LaserCan m_IntakeForwardLaserCan = new LaserCan(GlobalSensors.kIntakeForwardLaserCanID);
  LaserCan m_IntakeRearLaserCan = new LaserCan(GlobalSensors.kIntakeRearLaserCanID);

  public final Trigger laserCanTrigger_FORWARD =
      new Trigger(() -> (m_IntakeForwardLaserCan.getMeasurement().distance_mm < 100));

  public final Trigger laserCanTrigger_REAR =
      new Trigger(() -> (m_IntakeRearLaserCan.getMeasurement().distance_mm < 100));


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor = new CANSparkFlex(IntakeConstants.kMotorID, MotorType.kBrushless);

    m_intakeMotor.restoreFactoryDefaults();

    m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);

    m_intakeMotor.burnFlash();

    m_IntakeForwardLaserCan.setRangingMode(RangingMode.SHORT);
    m_IntakeForwardLaserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
    m_IntakeForwardLaserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);

    m_IntakeRearLaserCan.setRangingMode(RangingMode.SHORT);
    m_IntakeRearLaserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
    m_IntakeRearLaserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);

  }

  /* Commands */
  public Command intakeIntake() {
    return run(() -> setSpeed(INTAKE_SPEED.get()));
  }

  public Command intakeOuttake() {
    return run(() -> setSpeed(OUTTAKE_SPEED.get()));
  }

  public Command intakeTransferFwd() {
    return run(() -> setSpeed(TRANSFER_FORWARD_SPEED.get()));
  }

  public Command intakeTransferBck() {
    return run(() -> setSpeed(TRANSFER_BACKWARD_SPEED.get()));
  }

  public Command intakeStop() {
    return runOnce(() -> setSpeed(0));
  }

  /* Methods */
  public void setSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
