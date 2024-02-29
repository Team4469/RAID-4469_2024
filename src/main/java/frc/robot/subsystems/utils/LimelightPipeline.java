// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.utils;

/** Add your docs here. */
public enum LimelightPipeline {
  LOCALIZATION(0),
  SHOOT(1),
  STAGE(2),
  AMP(5);

  private int value;

  private LimelightPipeline(int value) {
    this.value = value;
  }

  public int getValue() {
    return value;
  }
}
