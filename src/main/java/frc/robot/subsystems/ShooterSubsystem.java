// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.ShooterConstants;
import frc.utils.TunableNumber;

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

    m_rightShooterEncoder = m_rightShooterMotor.getEncoder();
    m_leftShooterEncoder = m_leftShooterMotor.getEncoder();

    m_rightPIDController = m_rightShooterMotor.getPIDController();
    m_leftPIDController = m_leftShooterMotor.getPIDController();

    m_rightShooterMotor.restoreFactoryDefaults();
    m_leftShooterMotor.restoreFactoryDefaults();

    m_rightShooterMotor.setInverted(ShooterConstants.kRightMotorInverted);
    m_leftShooterMotor.setInverted(ShooterConstants.kLeftMotorInverted);

    m_rightShooterMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
    m_leftShooterMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);

    m_rightShooterMotor.burnFlash();
    m_leftShooterMotor.burnFlash();

    m_rightPIDController.setP(ShooterConstants.kP_right);
    m_rightPIDController.setI(ShooterConstants.kI_right);
    m_rightPIDController.setD(ShooterConstants.kD_right);
    m_rightPIDController.setIZone(ShooterConstants.kIz_right);
    m_rightPIDController.setFF(ShooterConstants.kFF_right);
    m_rightPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    m_leftPIDController.setP(ShooterConstants.kP_left);
    m_leftPIDController.setI(ShooterConstants.kI_left);
    m_leftPIDController.setD(ShooterConstants.kD_left);
    m_leftPIDController.setIZone(ShooterConstants.kIz_left);
    m_leftPIDController.setFF(ShooterConstants.kFF_left);
    m_leftPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
  }

  /* Command Factory */
  public Command shooterSpeakerShot() {
    return Commands.runOnce(() -> shootPIDControl(SHOOTER_SPEED_CLOSED_LOOP.get()));
  }

  public Command shooterStop() {
    return Commands.runOnce(this::shootStop);
  }

  public Command shooterFeed() {
    return Commands.runOnce(() -> setSpeed(SHOOTER_SPEED_OPEN_LOOP.get()));
  }

  public Command shooterAmpSmartCommand(AmpDirection ampDirection) {
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

  /* Methods */

  private void setSpeed(double speed) {
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
  }
}
