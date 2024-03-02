// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public class MAXSwerveModule {
  private final CANSparkFlex m_drivingSparkFlex;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkFlex = new CANSparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkFlex.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkFlex.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkFlex.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);


    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    for (int i = 0; i < 6; i++) {
      if (m_drivingEncoder.getPositionConversionFactor() != ModuleConstants.kDrivingEncoderPositionFactor) {
        m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
      } else {
        break;
      }
      Timer.delay(.1);
    }
    for (int i = 0; i < 6; i++) {
      if (m_drivingEncoder.getVelocityConversionFactor() != ModuleConstants.kDrivingEncoderVelocityFactor) {
        m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    for (int i = 0; i < 6; i++) {
      if (m_turningEncoder.getPositionConversionFactor() != ModuleConstants.kTurningEncoderPositionFactor) {
        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
      } else {
        break;
      }
      Timer.delay(.1);
    }
    for (int i = 0; i < 6; i++) {
      if (m_turningEncoder.getVelocityConversionFactor() != ModuleConstants.kTurningEncoderVelocityFactor) {
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    for (int i = 0; i < 6; i++) {
      if (m_turningEncoder.getInverted() != ModuleConstants.kTurningEncoderInverted) {
        m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(
        ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(
        ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    for (int i = 0; i < 6; i++) {
      if (m_drivingPIDController.getI() != ModuleConstants.kDrivingI) {
        m_drivingPIDController.setI(ModuleConstants.kDrivingI);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_drivingPIDController.getP() != ModuleConstants.kDrivingP) {
        m_drivingPIDController.setP(ModuleConstants.kDrivingP);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_drivingPIDController.getD() != ModuleConstants.kDrivingD) {
        m_drivingPIDController.setD(ModuleConstants.kDrivingD);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_drivingPIDController.getFF() != ModuleConstants.kDrivingFF) {
        m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    m_drivingPIDController.setOutputRange(
        ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    for (int i = 0; i < 6; i++) {
      if (m_turningPIDController.getI() != ModuleConstants.kTurningI) {
        m_turningPIDController.setI(ModuleConstants.kTurningI);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_turningPIDController.getP() != ModuleConstants.kTurningP) {
        m_turningPIDController.setP(ModuleConstants.kTurningP);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_turningPIDController.getD() != ModuleConstants.kTurningD) {
        m_turningPIDController.setD(ModuleConstants.kTurningD);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    for (int i = 0; i < 6; i++) {
      if (m_turningPIDController.getFF() != ModuleConstants.kTurningFF) {
        m_turningPIDController.setFF(ModuleConstants.kTurningFF);
      } else {
        break;
      }
      Timer.delay(.1);
    }

    m_turningPIDController.setOutputRange(
        ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    m_drivingSparkFlex.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkFlex.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);


    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // Output, Faults, Sticky Faults, Is Follower
    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // Motor Velo, Motor Temp, Motor Volts, Motor Current
    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // Motor Position
    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); // Alternate Encoder Velocity, Alternate Encoder Position
    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); // Absolute Encoder Position, Absolute Encoder Angle
    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); // Absolute Encoder Velocity, Absolute Encoder Frequency

    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // Output, Faults, Sticky Faults, Is Follower
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500); // Motor Velo, Motor Temp, Motor Volts, Motor Current
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // Motor Position
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); // Alternate Encoder Velocity, Alternate Encoder Position
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 100); // Absolute Encoder Position, Absolute Encoder Angle
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 100); // Absolute Encoder Velocity, Absolute Encoder Frequency


    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    Timer.delay(.25);

    m_drivingSparkFlex.burnFlash();
    m_turningSparkMax.burnFlash();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public boolean getDrivingCanRxFault() {
    return m_drivingSparkFlex.getFault(FaultID.kCANRX);
  }

  public boolean getDrivingCanTxFault() {
    return m_drivingSparkFlex.getFault(FaultID.kCANTX);
  }

  public double getDrivingCurrent() {
    return m_drivingSparkFlex.getOutputCurrent();
  }

  public double getDrivingOutput() {
    return m_drivingSparkFlex.getAppliedOutput();
  }

  public double getBusVoltage() {
    return m_drivingSparkFlex.getBusVoltage();
  }
}
