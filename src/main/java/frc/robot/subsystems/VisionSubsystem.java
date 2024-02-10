// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  LimelightHelpers.LimelightResults m_frontLimelightResults;
  LimelightHelpers.LimelightResults m_rearLimelightResults;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.kFrontLimelightName);
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.kRearLimelightName);
  }

  public double FrontTimestamp() {
    double latency_capture_mills = m_frontLimelightResults.targetingResults.latency_capture;
    double latency_pipline_mills = m_frontLimelightResults.targetingResults.latency_pipeline;
    return Timer.getFPGATimestamp()
        - (latency_capture_mills / 1000.0)
        - (latency_pipline_mills / 1000.0);
  }

  public double RearTimestamp() {
    double latency_capture_mills = m_rearLimelightResults.targetingResults.latency_capture;
    double latency_pipline_mills = m_rearLimelightResults.targetingResults.latency_pipeline;
    return Timer.getFPGATimestamp()
        - (latency_capture_mills / 1000.0)
        - (latency_pipline_mills / 1000.0);
  }

  public Pose2d FrontBotpose() {
    return m_frontLimelightResults.targetingResults.getBotPose2d_wpiBlue();
  }

  public Pose2d RearBotpose() {
    return m_rearLimelightResults.targetingResults.getBotPose2d_wpiBlue();
  }

  public boolean FrontValid() {
    return m_frontLimelightResults.targetingResults.valid;
  }

  public boolean RearValid() {
    return m_rearLimelightResults.targetingResults.valid;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
