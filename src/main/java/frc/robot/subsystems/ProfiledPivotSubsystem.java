// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import frc.utils.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LevatatorPivotConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;



public class ProfiledPivotSubsystem extends ProfiledPIDSubsystem {
  
  private final CANSparkFlex LPMotorL = new CANSparkFlex(LevatatorPivotConstants.kLPMotorL, MotorType.kBrushless);
  private final CANSparkFlex LPMotorR = new CANSparkFlex(LevatatorPivotConstants.kLPMotorR, MotorType.kBrushless);
  private final AbsoluteEncoder LPEncoder = LPMotorL.getAbsoluteEncoder(Type.kDutyCycle);
  
  /** Creates a new LevatatorPivotPID. */
  public ProfiledPivotSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            LevatatorPivotConstants.kP,
            LevatatorPivotConstants.kI,
            LevatatorPivotConstants.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(LevatatorPivotConstants.kMinOutput, LevatatorPivotConstants.kMaxOutput)));
      

      
      LPMotorL.restoreFactoryDefaults();
      LPMotorR.restoreFactoryDefaults();
  
      LPMotorL.setIdleMode(IdleMode.kCoast);
      LPMotorR.setIdleMode(IdleMode.kCoast);
  
      LPMotorL.setSmartCurrentLimit(LevatatorPivotConstants.kSmartCurrentLimit);
      LPMotorR.setSmartCurrentLimit(LevatatorPivotConstants.kSmartCurrentLimit);
  
      LPMotorR.follow(LPMotorL);

  
      LPMotorL.burnFlash();
      LPMotorR.burnFlash(); 
        
  }


    // Create variables
    //private final CANSparkFlex LPMotorL;
    //private final CANSparkFlex LPMotorR;
    
  
    TunableNumber PIVOT_SPEED =
    new TunableNumber("Intake/Pivot", LevatatorPivotConstants.kSpeedDefault);
  
    
    public Command RunShooterCommand() {
      return run(() -> MotorsOn(PIVOT_SPEED.get()));
    }
  
    public Command StopShooterCommand() {
      return runOnce(this::MotorsOff);
    }
  
    public void MotorsOn(double speed){
      if(Math.abs(speed) > 1) {speed = 1 * Math.signum(speed);}
      LPMotorL.set(speed);
    }
  
    public void MotorsOff(){
      LPMotorL.set(0);
    }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }


  @Override
  public double getMeasurement() {
    // Return the process variable measurement her
    return LPEncoder.getPosition();
  }

   public Command LPRotationGoalCommand(){
    return Commands.runOnce(
      () -> {
            this.setGoal(LevatatorPivotConstants.kLPRotationGoal);
            this.enable();
      },
    this);
  }

}
