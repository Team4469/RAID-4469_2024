// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.IntakeConstants;
import frc.utils.TunableNumber;
import monologue.Annotations.Log;

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

  // public final Trigger laserCanTrigger_REAR =
  //     new Trigger(
  //         () ->
  //             (m_IntakeRearLaserCan.getMeasurement().distance_mm
  //                 < IntakeConstants.kDetectionDistanceMM));

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor = new CANSparkFlex(IntakeConstants.kMotorID, MotorType.kBrushless);

    m_intakeMotor.restoreFactoryDefaults();

    m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);

    m_intakeMotor.setInverted(IntakeConstants.kMotorInverted);

    m_intakeMotor.burnFlash();

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

    m_intakeMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus0, 500); // Output, Faults, Sticky Faults, Is Follower
    m_intakeMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus1, 500); // Motor Velo, Motor Temp, Motor Volts, Motor Current
    m_intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // Motor Position
    m_intakeMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus3,
        500); // Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_intakeMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus4, 500); // Alternate Encoder Velocity, Alternate Encoder Position
    m_intakeMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus5, 500); // Absolute Encoder Position, Absolute Encoder Angle
    m_intakeMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus6, 500); // Absolute Encoder Velocity, Absolute Encoder Frequency
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
                .withTimeout(3))
                // .until(() -> (laserCanTrigger_REAR.getAsBoolean())))
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

  /* Commands */
  public Command intakePrepShoot() {
    Debouncer debounce =
        new Debouncer(IntakeConstants.kSensorDebounceTime, Debouncer.DebounceType.kRising);
    return runOnce(
            () -> {
              debounce.calculate(false);
            })
        // set the intake to intaking speed
        .andThen(
            run(() -> {
                  setSpeed(-.08);
                })
                // Wait until trigger is detected for more than 0.25s
                .until(() -> (laserCanTrigger_FORWARD.getAsBoolean())))
        .andThen(
            run(() -> {
                  setSpeed(.25);
                })
                // Wait until trigger is detected for more than 0.25s
                .withTimeout(.04))
        .finallyDo(
            (interrupted) -> {
              setSpeed(0);
            });
  }

  public Command intakePrepTrap() {
    Debouncer debounce =
        new Debouncer(IntakeConstants.kSensorDebounceTime, Debouncer.DebounceType.kRising);
    return run(() -> {
          setSpeed(-.1);
        })
        // Wait until trigger is detected for more than 0.25s
        .withTimeout(.4)
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
                  setSpeed(1);
                })
                // Wait until trigger is detected for more than 0.25s
                .withTimeout(1))
        // stop motor power
        .finallyDo(
            (interrupted) -> {
              setSpeed(0);
            });
  }

  public Command intakeAmpSmartCommand(AmpDirection ampSelect) {
    var amp = ampSelect;
    SmartDashboard.putString("Int Amp Dir", "" + amp);

    double speed;
    switch (amp) {
      case FRONT:
        speed = OUTTAKE_SPEED.get();
        break;
      default:
        speed = TRANSFER_FORWARD_SPEED.get();
        break;
    }
    return Commands.run(() -> setSpeed(speed));
  }

  public Command moveNoteCommand() {
    Debouncer debounce =
        new Debouncer(IntakeConstants.kSensorDebounceTime, Debouncer.DebounceType.kRising);
    return runOnce(
            () -> {
              debounce.calculate(false);
            })
        // set the intake to intaking speed
        .andThen(
            run(() -> {
                  setSpeed(TRANSFER_FORWARD_SPEED.get());
                })
                // Wait until trigger is detected for more than 0.25s
                .withTimeout(.4))
                // .until(() -> (laserCanTrigger_REAR.getAsBoolean())))
        // stop motor power
        .finallyDo(
            (interrupted) -> {
              setSpeed(0);
            });
  }

  public Command intakeIntake() {
    return Commands.run(() -> setSpeed(1));
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

  @Log
  private boolean getCanTxFault() {
    return m_intakeMotor.getFault(FaultID.kCANTX);
  }

  @Log
  private boolean getCanRxFault() {
    return m_intakeMotor.getFault(FaultID.kCANRX);
  }

  @Log
  private double getAppliedOutput() {
    return m_intakeMotor.getAppliedOutput();
  }

  @Log
  private double getCurrent() {
    return m_intakeMotor.getOutputCurrent();
  }

  @Log
  private double getBusVoltage() {
    return m_intakeMotor.getBusVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
