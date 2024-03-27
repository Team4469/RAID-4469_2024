// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ClimberModule;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CLIMBER_TRAP extends ParallelCommandGroup {
  /** Creates a new CLIMBER_TRAP. */
  public CLIMBER_TRAP(ClimberModule lClimber, ClimberModule rClimber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CLIMBER_TO_BOTTOM(lClimber), new CLIMBER_TO_BOTTOM(rClimber));
  }
}
