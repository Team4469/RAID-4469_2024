// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
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

  TunableNumber INTAKE_SPEED = new TunableNumber("Intake/Intake speed", .5);
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

    m_intakeMotor.setInverted(IntakeConstants.kMotorInverted);

    m_intakeMotor.burnFlash();

    m_intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, IntakeConstants.kStatus3PeriodMs);
    m_intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, IntakeConstants.kStatus4PeriodMs);
    m_intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, IntakeConstants.kStatus5PeriodMs);
    m_intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, IntakeConstants.kStatus6PeriodMs);

    try {
      m_IntakeForwardLaserCan.setRangingMode(RangingMode.SHORT);
      m_IntakeForwardLaserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
      m_IntakeForwardLaserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);

      m_IntakeRearLaserCan.setRangingMode(RangingMode.SHORT);
      m_IntakeRearLaserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
      m_IntakeRearLaserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  /* Commands */
  public Command intakeAutoIntake() {
    Debouncer debounce =
        new Debouncer(IntakeConstants.kSensorDebounceTime, Debouncer.DebounceType.kRising);
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
                .until(() -> (laserCanTrigger_REAR.getAsBoolean())))
        .andThen(
            run(() -> {
                  setSpeed(TRANSFER_BACKWARD_SPEED.get());
                })
                // Wait until trigger is detected for more than 0.25s
                .withTimeout(IntakeConstants.kAutoIntakeBackMoveTime))
        // stop motor power
        .finallyDo(
            (interrupted) -> {
              setSpeed(0);
            });
  }

  public Command intakeShootCommand() {
    Debouncer debounce =
        new Debouncer(IntakeConstants.kSensorDebounceTime, Debouncer.DebounceType.kRising);
    return runOnce(
            () -> {
              debounce.calculate(false);
            })
        // set the intake to backward transfer speed
        .andThen(
            run(() -> {
                  this.setSpeed(-.075);
                })
                // Wait until trigger is detected for more than 0.25s
                // .until(() -> (laserCanTrigger_FORWARD.getAsBoolean())))
                .withTimeout(.2))
        .andThen(
            run(() -> {
                  setSpeed(1);
                })
                // Wait until trigger is detected for more than 0.25s
                .withTimeout(2))
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
    return Commands.runOnce(() -> setSpeed(-1));
  }

  public Command intakeTransferFwd() {
    return Commands.runOnce(() -> setSpeed(TRANSFER_FORWARD_SPEED.get()));
  }

  public Command intakeTransferBck() {
    return Commands.runOnce(() -> setSpeed(TRANSFER_BACKWARD_SPEED.get()));
  }

  public Command intakeStop() {
    return Commands.runOnce(() -> setSpeed(0));
  }

  /* Methods */
  public void setSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  public void stop() {
    m_intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
