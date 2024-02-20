// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class SetPoints {
  public final class PivotSetpoints {
    // Currently using pid control on the spark so this should be in rads to be consistent with
    // other subs
    public static final double kStowed = Units.degreesToRadians(15); // Degrees
    public static final double kAmpFront = Units.degreesToRadians(90);
    public static final double kAmpRear = Units.degreesToRadians(45);
    public static final double kTrap = Units.degreesToRadians(100);
    public static final double kSubwoofer = Units.degreesToRadians(25);
    public static final double kIntake = Units.degreesToRadians(5);
  }

  public final class WristSetpoints {
    // Currently using trap profile subsystem so this needs to be in rads
    public static final double kStowed = Units.degreesToRadians(90);
    public static final double kAmpFront = Units.degreesToRadians(135);
    public static final double kAmpRear = Units.degreesToRadians(75);
    public static final double kTrap = Units.degreesToRadians(155);
    public static final double kSubwoofer = Units.degreesToRadians(90);
    public static final double kIntake = Units.degreesToRadians(135);
  }

  public final class LevetatorSetpoints {
    public static final double kStowed = Units.inchesToMeters(1);
    public static final double kAmpFront = Units.inchesToMeters(10);
    public static final double kAmpRear = Units.inchesToMeters(10);
    public static final double kTrap = Units.inchesToMeters(20);
    public static final double kSubwoofer = Units.inchesToMeters(1);
    public static final double kIntake = Units.inchesToMeters(12);
    public static final double kMovement = Units.inchesToMeters(3);
    public static final double kPodium = Units.inchesToMeters(2);
  }

  public final class ClimberSetpoints {
    public static final double kTrapHeight = Units.inchesToMeters(23);
    public static final double kRetractedHeight = Units.inchesToMeters(0);
  }
}
