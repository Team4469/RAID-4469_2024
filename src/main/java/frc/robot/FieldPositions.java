// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** All positions should be referenced with Blue Alliance being 0,0 */
public class FieldPositions {
  public static class StagePositions {
    // Stage Positions should have the rotation where the front of the robot is facing away from the
    // stage core
    public static final Pose2d StageRight_Blue =
        new Pose2d(
            Units.inchesToMeters(129.138),
            Units.inchesToMeters(172.9),
            Rotation2d.fromDegrees(-120));
    public static final Pose2d StageLeft_Blue =
        new Pose2d(
            Units.inchesToMeters(194.138),
            Units.inchesToMeters(172.9),
            Rotation2d.fromDegrees(120));
    public static final Pose2d CenterStage_Blue =
        new Pose2d(
            Units.inchesToMeters(161.638), Units.inchesToMeters(229.2), Rotation2d.fromDegrees(0));
    public static final Pose2d StageRight_Red =
        new Pose2d(
            Units.inchesToMeters(194.138),
            Units.inchesToMeters(478.313),
            Rotation2d.fromDegrees(60));
    public static final Pose2d StageLeft_Red =
        new Pose2d(
            Units.inchesToMeters(129.138),
            Units.inchesToMeters(478.313),
            Rotation2d.fromDegrees(-60));
    public static final Pose2d CenterStage_Red =
        new Pose2d(
            Units.inchesToMeters(161.638),
            Units.inchesToMeters(422.022),
            Rotation2d.fromDegrees(180));
  }
}
