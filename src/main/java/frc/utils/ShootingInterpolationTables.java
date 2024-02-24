// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ShootingInterpolationTables {
  public class ShooterRPMTable {
    public static final InterpolatingDoubleTreeMap SHOOTER_RPM_INTERP_TABLE =
        new InterpolatingDoubleTreeMap();

    static {
      SHOOTER_RPM_INTERP_TABLE.put(1.0, 4000.0);
      SHOOTER_RPM_INTERP_TABLE.put(2.0, 5000.0);
      SHOOTER_RPM_INTERP_TABLE.put(3.0, 6000.0);
    }
  }

  public class ShooterSpeedTable {
    public static final InterpolatingDoubleTreeMap SHOOTER_SPEED_INTERP_TABLE =
        new InterpolatingDoubleTreeMap();

    static {
      SHOOTER_SPEED_INTERP_TABLE.put(1.0, 1.0);
      SHOOTER_SPEED_INTERP_TABLE.put(2.0, 1.0);
      SHOOTER_SPEED_INTERP_TABLE.put(3.0, 1.0);
    }
  }

  public class ShooterLaunchAngleTable {
    public static final InterpolatingDoubleTreeMap SHOOTER_LAUNCH_ANGLE_INTERP_TABLE =
        new InterpolatingDoubleTreeMap();

    static {
      SHOOTER_LAUNCH_ANGLE_INTERP_TABLE.put(0.5, 2.65);
      SHOOTER_LAUNCH_ANGLE_INTERP_TABLE.put(1.0, 2.775);
      SHOOTER_LAUNCH_ANGLE_INTERP_TABLE.put(2.0, 2.775 + Units.degreesToRadians(17));
      SHOOTER_LAUNCH_ANGLE_INTERP_TABLE.put(3.0, 2.775 + Units.degreesToRadians(20));
    }
  }
}
