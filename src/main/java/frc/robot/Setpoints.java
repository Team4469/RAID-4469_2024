// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Setpoints {
    public static final class LevetatorPivotSetpoints {

    public static final double kStowGoal = Units.degreesToRadians(90);
    public static final double kIntakeGoal = Units.degreesToRadians(135);
    public static final double kSubwooferGoal = Units.degreesToRadians(75);
    public static final double kAmpForwardGoal = Units.degreesToRadians(155);
    public static final double kAmpBackGoal = Units.degreesToRadians(135);
    public static final double kTrapGoal = Units.degreesToRadians(50);
    public static final double kLPRotationGoal = Units.degreesToRadians(10); //Temporary Value. Will be changed 
    }
}
