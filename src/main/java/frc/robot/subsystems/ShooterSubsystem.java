// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.ShooterConstants;
import frc.utils.ShootingInterpolationTables.ShooterRPMTable;
import frc.utils.ShootingInterpolationTables.ShooterSpeedTable;
import frc.utils.TunableNumber;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {

  TunableNumber SHOOTER_SPEED_CLOSED_LOOP = new TunableNumber("Shooter/RPMSetpoint", 0);

  TunableNumber SHOOTER_SPEED_OPEN_LOOP = new TunableNumber("Shooter/FeedSpeed", 0);

  private final CANSparkFlex m_rightShooterMotor;
  private final CANSparkFlex m_leftShooterMotor;

  private final RelativeEncoder m_rightShooterEncoder;
  private final RelativeEncoder m_leftShooterEncoder;

  private final SparkPIDController m_rightPIDController;
  private final SparkPIDController m_leftPIDController;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_rightShooterMotor =
        new CANSparkFlex(ShooterConstants.kRightShooterCanID, MotorType.kBrushless);
    m_leftShooterMotor = new CANSparkFlex(ShooterConstants.kLeftShooterCanID, MotorType.kBrushless);
    m_rightShooterMotor.restoreFactoryDefaults();
    m_leftShooterMotor.restoreFactoryDefaults();

    m_rightShooterEncoder =
        m_rightShooterMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
    m_leftShooterEncoder =
        m_leftShooterMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);

    m_rightPIDController = m_rightShooterMotor.getPIDController();
    m_leftPIDController = m_leftShooterMotor.getPIDController();

    m_rightPIDController.setFeedbackDevice(m_rightShooterEncoder);
    m_leftPIDController.setFeedbackDevice(m_leftShooterEncoder);

    // for (int i = 0; i < 6; i++) {
    //   if (m_rightShooterMotor.getInverted() != ShooterConstants.kRightMotorInverted) {
        m_rightShooterMotor.setInverted(ShooterConstants.kRightMotorInverted);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_leftShooterMotor.getInverted() != ShooterConstants.kLeftMotorInverted) {
        m_leftShooterMotor.setInverted(ShooterConstants.kLeftMotorInverted);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    m_rightShooterMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
    m_leftShooterMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);

    m_leftShooterMotor.setIdleMode(IdleMode.kBrake);
    m_rightShooterMotor.setIdleMode(IdleMode.kBrake);

    // m_rightShooterEncoder.setAverageDepth(8);
    // m_rightShooterEncoder.setMeasurementPeriod(32);

    // m_leftShooterEncoder.setAverageDepth(8);
    // m_leftShooterEncoder.setMeasurementPeriod(32);

    // for (int i = 0; i < 6; i++) {
    //   if (m_leftPIDController.getI() != ShooterConstants.kI_left) {
        m_leftPIDController.setI(ShooterConstants.kI_left);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_leftPIDController.getP() != ShooterConstants.kP_left) {
        m_leftPIDController.setP(ShooterConstants.kP_left);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_leftPIDController.getD() != ShooterConstants.kD_left) {
        m_leftPIDController.setD(ShooterConstants.kD_left);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }
    m_rightPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    // for (int i = 0; i < 6; i++) {
    //   if (m_leftPIDController.getI() != ShooterConstants.kI_right) {
        m_leftPIDController.setI(ShooterConstants.kI_right);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_leftPIDController.getP() != ShooterConstants.kP_right) {
        m_leftPIDController.setP(ShooterConstants.kP_right);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    // for (int i = 0; i < 6; i++) {
    //   if (m_leftPIDController.getD() != ShooterConstants.kD_right) {
        m_leftPIDController.setD(ShooterConstants.kD_right);
    //   } else {
    //     break;
    //   }
    //   Timer.delay(.1);
    // }

    m_leftPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    m_leftShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus0, 500); // Output, Faults, Sticky Faults, Is Follower
    m_leftShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus1, 20); // Motor Velo, Motor Temp, Motor Volts, Motor Current
    m_leftShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // Motor Position
    m_leftShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus3,
        500); // Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_leftShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus4, 500); // Alternate Encoder Velocity, Alternate Encoder Position
    m_leftShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus5, 500); // Absolute Encoder Position, Absolute Encoder Angle
    m_leftShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus6, 500); // Absolute Encoder Velocity, Absolute Encoder Frequency

    m_rightShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus0, 500); // Output, Faults, Sticky Faults, Is Follower
    m_rightShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus1, 20); // Motor Velo, Motor Temp, Motor Volts, Motor Current
    m_rightShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // Motor Position
    m_rightShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus3,
        500); // Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_rightShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus4, 500); // Alternate Encoder Velocity, Alternate Encoder Position
    m_rightShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus5, 500); // Absolute Encoder Position, Absolute Encoder Angle
    m_rightShooterMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus6, 500); // Absolute Encoder Velocity, Absolute Encoder Frequency

    // Timer.delay(.25);

    m_rightShooterMotor.burnFlash();
    m_leftShooterMotor.burnFlash();
  }

  /* Command Factory */
  public Command shooterSpeakerShot() {
    return Commands.runOnce(() -> setSpeed(1));
  }

  public Command shooterSpeakerShotRPM() {
    return Commands.runOnce(() -> shootPIDControl(6000));
  }

  public Command shooterTrapCommand() {
    return Commands.runOnce(() -> setSpeed(1));
  }

  /**
   * @param distanceToTarget distance to target in Meters
   */
  public Command shooterVariableSpeakerShot(DoubleSupplier distanceToTarget) {
    double target = ShooterRPMTable.SHOOTER_RPM_INTERP_TABLE.get(distanceToTarget.getAsDouble());
    return Commands.run(() -> shootPIDControl(target));
  }

  /**
   * @param distanceToTarget distance to target in Meters
   */
  public Command shooterVariableSpeedSpeakerShot(DoubleSupplier distanceToTarget) {
    double target =
        ShooterSpeedTable.SHOOTER_SPEED_INTERP_TABLE.get(distanceToTarget.getAsDouble());
    SmartDashboard.putNumber("Target Speed", target);
    return Commands.run(() -> setSpeed(target));
  }

  // SHOOTER_SPEED_INTERP_TABLE

  public Command shooterStop() {
    return Commands.runOnce(this::shootStop);
  }

  public Command shooterTrap() {
    return Commands.runOnce(() -> setSpeed(.2));
  }

  public Command shooterFeed() {
    return Commands.runOnce(() -> setSpeed(.5));
  }

  public Command shooterChaos() {
    return Commands.runOnce(() -> setSpeed(.15));
  }

  public Command shooterAmpSmartCommand(AmpDirection ampSelect) {
    var ampDirection = ampSelect;
    double speed;
    switch (ampDirection) {
      case FRONT:
        speed = 0;
        break;
      default:
        speed = SHOOTER_SPEED_OPEN_LOOP.get();
        break;
    }
    return Commands.run(() -> setSpeed(speed));
  }

  public Command shooterAboveSpeedCommand() {
    return Commands.waitUntil(this::aboveSpeed);
  }

  /* Methods */

  private boolean aboveSpeed() {
    var currentSpeed =
        Math.min(m_rightShooterEncoder.getVelocity(), m_leftShooterEncoder.getVelocity());
    if (currentSpeed > 6200) {
      return true;
    } else {
      return false;
    }
  }

  public void setSpeed(double speed) {
    m_rightShooterMotor.set(speed);
    m_leftShooterMotor.set(speed);
  }

  private void shootStop() {
    m_rightShooterMotor.set(0);
    m_leftShooterMotor.set(0);
  }

  private void shootPIDControl(double setpoint) {
    m_leftPIDController.setReference(setpoint, ControlType.kVelocity);
    m_rightPIDController.setReference(setpoint, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Right Shooter RPM", m_rightShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Left Shooter RPM", m_leftShooterEncoder.getVelocity());

    // if (m_rightShooterMotor.getInverted() != ShooterConstants.kRightMotorInverted) {
    //   m_rightShooterMotor.setInverted(ShooterConstants.kRightMotorInverted);
    // }

    // if (m_leftShooterMotor.getInverted() != ShooterConstants.kLeftMotorInverted) {
    //   m_leftShooterMotor.setInverted(ShooterConstants.kLeftMotorInverted);
    // }
  }
}
