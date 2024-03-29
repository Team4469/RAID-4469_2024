// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FrontLimelightConstants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private boolean initialized = false;

  private NetworkTableEntry tTarget = null;
  private NetworkTableEntry tx = null;
  private NetworkTableEntry ty = null;
  private NetworkTableEntry ta = null;
  private NetworkTableEntry botpose = null;
  private NetworkTableEntry targetpose = null;
  private NetworkTableEntry tl = null;
  private NetworkTableEntry cl = null;
  private NetworkTableEntry pipeline = null;
  private NetworkTableEntry tid = null;
  private String limeLightName = "limelight";

  public Limelight(String limeLightName) {
    this.limeLightName = limeLightName;
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limeLightName);
    turnOffLED();
    try {
      tTarget = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      botpose = table.getEntry("botpose_wpiblue");
      targetpose = table.getEntry("targetpose_robotspace");
      tl = table.getEntry("tl");
      cl = table.getEntry("cl");
      pipeline = table.getEntry("pipeline");
      tid = table.getEntry("tid");
    } catch (Exception e) {
      // SmartDashboard.putBoolean("couldn't get nt entries", true);
    }
    initialized = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limeLightName);
    try {
      tTarget = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      botpose = table.getEntry("botpose_wpiblue");
      targetpose = table.getEntry("targetpose_robotspace");
      tl = table.getEntry("tl");
      cl = table.getEntry("cl");
      pipeline = table.getEntry("pipeline");
      tid = table.getEntry("tid");
    } catch (Exception e) {
      return;
    }
    // SmartDashboard.putString("botpose nt", botpose.toString());
  }

  public NetworkTableEntry getEntry(String str) {
    return NetworkTableInstance.getDefault().getTable(limeLightName).getEntry(str);
  }

  public boolean isInitialized() {
    return this.initialized;
  }

  public boolean hasTargets() {
    boolean hits = false;
    // SmartDashboard.putBoolean("isInitialized", isInitialized());
    if (isInitialized()) {
      hits = (getEntry("tv").getDouble(0.0) == 1.0);
    }
    return hits;
  }

  public Pose2d botPose() {
    double[] botPose = null;
    // SmartDashboard.putBoolean("Limelight Inititialized", isInitialized());
    if (isInitialized()) {
      botPose = botpose.getDoubleArray(new double[7]);
    }
    return toPose2D(botPose);
  }

  public double tl() {
    double tL = 0.0;
    if (isInitialized()) {
      tL = tl.getDouble(0.0);
    }
    return tL;
  }

  public double cl() {
    double cL = 0.0;
    if (isInitialized()) {
      cL = cl.getDouble(0.0);
    }
    return cL;
  }

  public double getTargetInView() {
    double tId = 0.0;
    if (isInitialized()) {
      tId = tid.getDouble(0.0);
    }
    return tId;
  }

  public double targetDist() {
    double[] targetPose = null;
    if (isInitialized()) {
      targetPose = targetpose.getDoubleArray(new double[3]);
    }
    try {
      Translation3d dist = new Translation3d(targetPose[0], targetPose[1], targetPose[2]);
      return dist.getDistance(new Translation3d());
    } catch (Exception e) {
      // TODO: handle exception
      return 0;
    }
  }

  private static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      System.err.println("Bad LL 2D Pose Data!");
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    return new Pose2d(tran2d, r2d);
  }

  public double x() {
    double dx = 0.0;
    if (isInitialized()) {
      dx = tx.getDouble(0.0);
    }
    return dx;
  }

  public double y() {
    double dy = 0.0;
    if (isInitialized()) {
      dy = ty.getDouble(0.0);
    }
    return dy;
  }

  public double targetArea() {
    double dArea = 0.0;
    if (isInitialized()) {
      dArea = ta.getDouble(0.0);
    }
    return dArea;
  }

  public double tv() {
    double tv = 0.0;
    if (isInitialized()) {
      tv = tTarget.getDouble(0.0);
    }
    return tv;
  }

  public void turnOnLED() {
    lightLED(LimelightLED.ON);
  }

  public void turnOffLED() {
    lightLED(LimelightLED.OFF);
  }

  private void lightLED(LimelightLED value) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limeLightName);
    table.getEntry("ledMode").setNumber(value.ordinal());
  }

  private void setPipeline(LimelightPipeline pipeline) {
    int pipe = pipeline.getValue();
    if (isInitialized()) {
      this.pipeline.setNumber(pipe);
    }
    // NetworkTableInstance.getDefault().getTable(limeLightName).getEntry("pipeline").setNumber(pipe);
  }

  public Command setPipelineCommand(LimelightPipeline pipeline) {
    return runOnce(() -> setPipeline(pipeline));
  }

  public double SimpleDistanceToSpeakerMeters() {

    double targetOffsetAngle_Vertical = y();

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = FrontLimelightConstants.kAngleFromVerticalDegrees;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightMeters = FrontLimelightConstants.kDistanceFromFloorMeters;

    // distance from the target to the floor
    double goalHeightMeters = Units.inchesToMeters(57.125);

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = Units.degreesToRadians(angleToGoalDegrees);

    // calculate distance
    double distanceFromLimelightToGoalMeters =
        (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalMeters;
  }

  public double limelight_strafe_x_proportional() {
    double kP = .003;
    double targetXSpeed = this.x() * kP;

    // targetXSpeed *= -1.0;

    return targetXSpeed;
  }

  public double limelight_strafe_y_proportional() {
    double kP = .004;
    double targetYSpeed = this.y() * kP;

    targetYSpeed *= -1.0;

    return targetYSpeed;
  }

  public double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = this.y() * kP;
    var tv = this.tv();

    if (tv == 0) {
      // if no target, we want to spin in place so no forward speed
      targetingForwardSpeed = 0;
    } else {
      // invert due to limelight results
      targetingForwardSpeed *= -1.0;
    }

    return targetingForwardSpeed;
  }

  public boolean limelight_in_range() {
    var tv = this.hasTargets();
    var ty = this.y();
    var tx = this.x();

    if (tv && Math.abs(ty) < 2.0 && Math.abs(tx) < 2.0) {
      return true;
    } else {
      return false;
    }
  }

  public boolean limelight_x_in_range() {
    var tx = this.x();
    var tv = this.hasTargets();

    if (tv && Math.abs(tx) < 2.0) {
      return true;
    } else {
      return false;
    }
  }

  public final Trigger shooterTargetInRange = new Trigger(this::limelight_x_in_range);
}
