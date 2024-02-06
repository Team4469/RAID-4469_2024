// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.IntakeConstants;
import frc.utils.TunableNumber;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkFlex m_intakeMotor;

  TunableNumber INTAKE_SPEED = new TunableNumber("Intake/Intake speed", .75);
  TunableNumber OUTTAKE_SPEED =
      new TunableNumber("Intake/Outtake speed", -.85); // FOR AMP FROM FRONT
  TunableNumber TRANSFER_FORWARD_SPEED = new TunableNumber("Intake/Transfer fwd speed", .2);
  TunableNumber TRANSFER_BACKWARD_SPEED = new TunableNumber("Intake/Transfer bck speed", -.2);

  LaserCan m_IntakeForwardLaserCan = new LaserCan(IntakeConstants.kIntakeForwardLaserCanID);
  LaserCan m_IntakeRearLaserCan = new LaserCan(IntakeConstants.kIntakeRearLaserCanID);

  public final Trigger laserCanTrigger_FORWARD =
      new Trigger(
          () ->
              (m_IntakeForwardLaserCan.getMeasurement().distance_mm
                  < IntakeConstants.kDetectionDistanceMM));

  public final Trigger laserCanTrigger_REAR =
      new Trigger(
          () ->
              (m_IntakeRearLaserCan.getMeasurement().distance_mm
                  < IntakeConstants.kDetectionDistanceMM));

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
  public Command intakeAutoIntake() {
    Debouncer debounce = new Debouncer(.25, Debouncer.DebounceType.kRising);
    return runOnce(
            () -> {
              debounce.calculate(false);
            })
        // set the intake to intaking speed
        .andThen(
            run(() -> {
                  setSpeed(INTAKE_SPEED.get());
                })
                // Wait until trigger is detected for more than 0.25s
                .until(() -> debounce.calculate(laserCanTrigger_REAR.getAsBoolean())))
        .andThen(
            runOnce(
                () -> {
                  debounce.calculate(false);
                }))
        .andThen(
            run(() -> {
                  setSpeed(TRANSFER_BACKWARD_SPEED.get());
                })
                // Wait until trigger is detected for more than 0.25s
                .until(() -> debounce.calculate(laserCanTrigger_FORWARD.getAsBoolean())))
        // stop motor power
        .finallyDo(
            (interrupted) -> {
              setSpeed(0);
            });
  }

  public Command intakeAmpSmartCommand(AmpDirection ampDirection) {
    double speed;
    switch (ampDirection) {
      case FRONT:
        speed = OUTTAKE_SPEED.get();
        break;
      default:
        speed = TRANSFER_FORWARD_SPEED.get();
        break;
    }
    return Commands.run(() -> setSpeed(speed));
  }

  public Command intakeIntake() {
    return Commands.run(() -> setSpeed(INTAKE_SPEED.get()));
  }

  public Command intakeOuttake() {
    return Commands.run(() -> setSpeed(OUTTAKE_SPEED.get()));
  }

  public Command intakeTransferFwd() {
    return Commands.run(() -> setSpeed(TRANSFER_FORWARD_SPEED.get()));
  }

  public Command intakeTransferBck() {
    return Commands.run(() -> setSpeed(TRANSFER_BACKWARD_SPEED.get()));
  }

  public Command intakeStop() {
    return Commands.runOnce(() -> setSpeed(0));
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
