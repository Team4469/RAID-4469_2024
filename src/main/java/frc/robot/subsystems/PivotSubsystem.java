// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.PivotConstants;
import frc.robot.SetPoints.PivotSetpoints;

public class PivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_leadMotor;
  private final CANSparkMax m_followMotor;
  private final AbsoluteEncoder m_encoder;

  private final SparkPIDController m_pidController;

  private double SETPOINT;
  private boolean SETPOINT_INIT;
  private int init_loop_number = 15;
  private int init_loop_count = 0;
  MedianFilter Setpoint_init_filter = new MedianFilter(init_loop_number);

  /** Creates a new PivotSubsystem2. */
  public PivotSubsystem() {

    m_leadMotor = new CANSparkMax(PivotConstants.kLeadMotorPort, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(PivotConstants.kFollowerMotorPort, MotorType.kBrushless);

    m_leadMotor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();

    m_pidController = m_leadMotor.getPIDController();

    m_pidController.setP(PivotConstants.kP); // .4
    m_pidController.setI(PivotConstants.kI); // 0
    m_pidController.setD(PivotConstants.kD); // 14

    m_pidController.setPositionPIDWrappingEnabled(false);

    m_pidController.setPositionPIDWrappingMaxInput(4);
    m_pidController.setPositionPIDWrappingMinInput(Math.PI / 2);

    m_pidController.setOutputRange(PivotConstants.kMinOutput, PivotConstants.kMaxOutput);

    m_leadMotor.setClosedLoopRampRate(PivotConstants.kClosedLoopRampRate);

    m_encoder = m_leadMotor.getAbsoluteEncoder(Type.kDutyCycle);

    m_pidController.setFeedbackDevice(m_encoder);

    m_leadMotor.setInverted(PivotConstants.kLeadMotorInverted);

    m_leadMotor.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);
    m_followMotor.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);

    m_leadMotor.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.kForwardSoftLimit);
    m_leadMotor.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.kReverseSoftLimit);
    m_leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_encoder.setZeroOffset(PivotConstants.kPivotZeroOffset);

    m_followMotor.follow(m_leadMotor, PivotConstants.kFollowMotorInverted);

    m_leadMotor.setIdleMode(IdleMode.kBrake);
    m_followMotor.setIdleMode(IdleMode.kBrake);

    m_encoder.setPositionConversionFactor(PivotConstants.kPivotEncoderPositionFactor);
    m_encoder.setVelocityConversionFactor(PivotConstants.kPivotEncoderVelocityFactor);

    m_leadMotor.burnFlash();
    m_followMotor.burnFlash();

    // m_leadMotor.setPeriodicFramePeriod(
    //     PeriodicFrame.kStatus0, PivotConstants.kLeaderStatus0PeriodMs);
    // m_leadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, PivotConstants.kStatus3PeriodMs);
    // m_leadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, PivotConstants.kStatus4PeriodMs);
    // m_leadMotor.setPeriodicFramePeriod(
    //     PeriodicFrame.kStatus5, PivotConstants.kLeaderStatus5PeriodMs);

    // m_followMotor.setPeriodicFramePeriod(
    //     PeriodicFrame.kStatus0, PivotConstants.kFollowerStatus0PeriodMs);
    // m_followMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3,
    // PivotConstants.kStatus3PeriodMs);
    // m_followMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4,
    // PivotConstants.kStatus4PeriodMs);
    // m_followMotor.setPeriodicFramePeriod(
    //     PeriodicFrame.kStatus5, PivotConstants.kFollowerStatus5PeriodMs);

    SETPOINT_INIT = false;
  }

  public Command pivotForward() {
    return run(this::speedFwd);
  }

  public Command pivotReverse() {
    return run(this::speedRev);
  }

  public Command pivotStop() {
    return runOnce(this::speedStop);
  }

  public double getRadiansFromHorizontal() {
    return m_encoder.getPosition() - Math.PI / 2;
  }

  public Command pivotInRange() {
    return Commands.waitUntil(() -> inRange(getSetpoint()));
  }

  public Command pivotAmpSmartCommand(AmpDirection ampDirection) {
    double point;
    switch (ampDirection) {
      case FRONT:
        point = PivotSetpoints.kAmpFront;
        break;
      default:
        point = PivotSetpoints.kAmpRear;
        break;
    }
    return Commands.run(
            () -> {
              setAngle(point);
            })
        .until(() -> inRange(point));
  }

  public Command pivotSetpointCommand(double radians) {
    double setpoint = radians;
    return Commands.runOnce(() -> setSetpoint(setpoint));
  }

  // public Command pivotSetpointVerticalCommand() {
  //   System.out.println("Pivot set to 3.14");
  //   return Commands.runOnce(() -> setSetpoint(3.14));
  // }

  // public Command pivotSetpoint45Command() {
  //   System.out.println("Pivot set to 2.18");
  //   return Commands.runOnce(() -> setSetpoint(2.18));
  // }

  private void setAngle(double radians) {
    m_pidController.setReference(radians, ControlType.kPosition);
  }

  public boolean inRange(double setpoint) {
    double measurement = getMeasurement();
    if (setpoint > measurement - Units.degreesToRadians(2)
        && setpoint < measurement + Units.degreesToRadians(2)) {
      return true;
    } else {
      return false;
    }
  }

  private double getMeasurement() {
    return m_encoder.getPosition();
  }

  public void speedFwd() {
    m_leadMotor.set(.1);
  }

  public void speedRev() {
    m_leadMotor.set(-.1);
  }

  public void speedStop() {
    m_leadMotor.set(0);
  }

  private void setSetpoint(double radians) {
    SETPOINT = radians;
    m_pidController.setReference(SETPOINT, ControlType.kPosition, 0, .2, ArbFFUnits.kVoltage);
  }

  private double getSetpoint() {
    return SETPOINT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!SETPOINT_INIT) {
      setSetpoint(Setpoint_init_filter.calculate(getMeasurement()));
      if (init_loop_count <= init_loop_number) {
        init_loop_count++;
      } else {
        SETPOINT_INIT = true;
      }
    }

    SmartDashboard.putNumber("Pivot Setpoint", getSetpoint());
    SmartDashboard.putNumber("Pivot Encoder", getMeasurement());
    SmartDashboard.putBoolean("Pivot In Range", inRange(getSetpoint()));
  }
}
