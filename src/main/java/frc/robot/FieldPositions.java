// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** All positions should be referenced with Blue Alliance being 0,0 */
public class FieldPositions {
  public static class StagePositions {
    // Stage Positions should have the rotation where the front of the robot is facing away from the
    // stage core
    public static final Pose2d StageRight_Blue =
        new Pose2d(
            Units.inchesToMeters(172.9),
            Units.inchesToMeters(129.138),
            Rotation2d.fromDegrees(-120));
    public static final Pose2d StageLeft_Blue =
        new Pose2d(
            Units.inchesToMeters(172.9),
            Units.inchesToMeters(194.138),
            Rotation2d.fromDegrees(120));
    public static final Pose2d CenterStage_Blue =
        new Pose2d(
            Units.inchesToMeters(229.2), Units.inchesToMeters(161.638), Rotation2d.fromDegrees(0));
    public static final Pose2d StageRight_Red =
        new Pose2d(
            Units.inchesToMeters(478.313),
            Units.inchesToMeters(194.138),
            Rotation2d.fromDegrees(60));
    public static final Pose2d StageLeft_Red =
        new Pose2d(
            Units.inchesToMeters(478.313),
            Units.inchesToMeters(129.138),
            Rotation2d.fromDegrees(-60));
    public static final Pose2d CenterStage_Red =
        new Pose2d(
            Units.inchesToMeters(422.022),
            Units.inchesToMeters(161.638),
            Rotation2d.fromDegrees(180));
  }

  public static class CloseToStagePoses {
    public static final Pose2d CloseSR_Blue = new Pose2d(4, 2.57, Rotation2d.fromDegrees(-120));
    public static final Pose2d CloseSL_Blue = new Pose2d(4, 5.6, Rotation2d.fromDegrees(120));
    public static final Pose2d CloseCS_Blue = new Pose2d(6.81, 4.11, Rotation2d.fromDegrees(0));
    public static final Pose2d CloseSR_Red = new Pose2d(12.62, 5.74, Rotation2d.fromDegrees(60));
    public static final Pose2d StageLeft_Red = new Pose2d(12.62, 2.57, Rotation2d.fromDegrees(-60));
    public static final Pose2d CenterStage_Red =
        new Pose2d(9.64, 4.11, Rotation2d.fromDegrees(180));
  }

  public static class StageTranslations {
    public static final Translation2d SL_BLUE_ADJUST =
        new Translation2d(Units.inchesToMeters(8), Rotation2d.fromDegrees(-60));

    public static final Translation2d SR_BLUE_ADJUST =
        new Translation2d(Units.inchesToMeters(8), Rotation2d.fromDegrees(60));

    public static final Translation2d CS_BLUE_ADJUST =
        new Translation2d(Units.inchesToMeters(8), Rotation2d.fromDegrees(180));

    public static final Translation2d SL_RED_ADJUST =
        new Translation2d(Units.inchesToMeters(8), Rotation2d.fromDegrees(120));

    public static final Translation2d SR_RED_ADJUST =
        new Translation2d(Units.inchesToMeters(8), Rotation2d.fromDegrees(-120));

    public static final Translation2d CS_RED_ADJUST =
        new Translation2d(Units.inchesToMeters(8), Rotation2d.fromDegrees(0));
  }

  public static final class SpeakerLocations {
    public static final Pose3d SPEAKER_BLUE_POSE3D =
        new Pose3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(80),
            new Rotation3d());
    public static final Pose3d SPEAKER_RED_POSE3D =
        new Pose3d(
            Units.inchesToMeters(651.23),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(80),
            new Rotation3d());
  }
}
