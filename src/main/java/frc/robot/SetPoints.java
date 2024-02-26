// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class SetPoints {
  public final class PivotSetpoints {
    // Currently using pid control on the spark so this should be in rads to be consistent with
    // other subs
    public static final double kStowed = 2.5;
    public static final double kAmpFront = 3.3;
    public static final double kAmpRear = 2.85;
    public static final double kTrap = Units.degreesToRadians(190);
    public static final double kSubwoofer = Units.degreesToRadians(155);
    public static final double kVariableShot = 2.5;
    public static final double kIntake = 1.58;
  }

  public final class WristSetpoints {
    // Currently using trap profile subsystem so this needs to be in rads
    public static final double kStowed = Units.degreesToRadians(180);
    public static final double kAmpFront = 3.9;
    public static final double kAmpRear = 3.14;
    public static final double kTrap = Units.degreesToRadians(215);
    public static final double kSubwoofer = Units.degreesToRadians(160);
    public static final double kIntake = 3.9;
  }

  public final class LevetatorSetpoints {
    public static final double kStowed = Units.inchesToMeters(.25);
    public static final double kAmpFront = .155;
    public static final double kAmpRear = .202;
    public static final double kTrap = Units.inchesToMeters(7);
    public static final double kSubwoofer = Units.inchesToMeters(1);
    public static final double kIntake = Units.inchesToMeters(4.25);
    public static final double kMovement = Units.inchesToMeters(3);
    public static final double kPodium = Units.inchesToMeters(2);
  }

  public final class ClimberSetpoints {
    public static final double kTrapHeight = Units.inchesToMeters(23);
    public static final double kRetractedHeight = Units.inchesToMeters(0);
  }

  public final class IntakeSetpoints {
    public static final double kTransferFwdFeedSpeed = .2;
    public static final double kTransferBckFeedSpeed = -.2;
    public static final double kAmpRearSpeed = -.85;
    public static final double kIntakeSpeed = .5;
  }

  public final class ShooterSetpoints {
    public static final double kAmpFrontSpeed = .5;
  }
}
