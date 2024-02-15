// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldPositions.SpeakerLocations;
import java.util.Optional;
import java.util.function.Supplier;

/** Add your docs here. */
public class ShootingCalculators {
  public static double DistanceToSpeakerMeters(Supplier<Pose2d> currentPose) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    Pose2d robotPose = currentPose.get();
    Pose3d speakerPose = SpeakerLocation(ally);
    Pose3d shooterPose = new Pose3d(robotPose);
    Transform3d shooterOnRobot = new Transform3d(0, 0, Units.inchesToMeters(12), null);
    shooterPose.transformBy(shooterOnRobot);

    var shooterToSpeakerDistance =
        shooterPose.getTranslation().getDistance(speakerPose.getTranslation());
    return shooterToSpeakerDistance;
  }

  public static Rotation2d RotationToSpeaker(Supplier<Pose2d> currentPose) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    Pose2d robotPose = currentPose.get();
    Pose3d speakerPose = SpeakerLocation(ally);
    Pose3d shooterPose = new Pose3d(robotPose);
    Transform3d shooterOnRobot = new Transform3d(0, 0, Units.inchesToMeters(12), null);
    shooterPose.transformBy(shooterOnRobot);

    Transform3d shooterToSpeaker = shooterPose.minus(speakerPose);
    Rotation2d shooterToSpeakerRotation2d = Rotation2d.fromRadians(shooterToSpeaker.getZ());
    return shooterToSpeakerRotation2d;
  }

  private static Pose3d SpeakerLocation(Optional<Alliance> alliance) {
    Pose3d loc;
    if (alliance.get() == Alliance.Red) {
      loc = SpeakerLocations.SPEAKER_RED_POSE3D;
    } else {
      loc = SpeakerLocations.SPEAKER_BLUE_POSE3D;
    }
    return loc;
  }
}
