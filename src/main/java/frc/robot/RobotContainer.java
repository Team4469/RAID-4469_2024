// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalSensors;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.DRIVE_WITH_HEADING;
import frc.robot.commands.drive.DRIVE_WITH_HEADING_SUPPLIER;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LevetatorSubsystem;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LevetatorSubsystem m_levetator = new LevetatorSubsystem();

  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  LaserCan m_FrontIntakeForwardLaserCan = new LaserCan(GlobalSensors.kFrontIntakeForwardLaserCanID);
  LaserCan m_FrontIntakeRearLaserCan = new LaserCan(GlobalSensors.kFrontIntakeRearLaserCanID);

  Trigger forwardLaserCanTrigger =
      new Trigger(
          () ->
              (m_FrontIntakeForwardLaserCan.getMeasurement().distance_mm > 10
                  && m_FrontIntakeForwardLaserCan.getMeasurement().distance_mm < 100));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("setX", m_robotDrive.setXCommand());

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true,
                    true),
            m_robotDrive));

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    m_FrontIntakeForwardLaserCan.setRangingMode(RangingMode.SHORT);
    m_FrontIntakeForwardLaserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
    m_FrontIntakeForwardLaserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController
        .rightBumper()
        .whileTrue(m_robotDrive.setXCommand());

    m_driverController
        .a()
        .onTrue(
            new DRIVE_WITH_HEADING(
                    m_robotDrive,
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    180)
                .until(() -> Math.abs(m_driverController.getRightX()) > 0.3));

    m_driverController
        .y()
        .onTrue(
            new DRIVE_WITH_HEADING(
                    m_robotDrive,
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    0)
                .until(() -> Math.abs(m_driverController.getRightX()) > 0.3));

    m_driverController
        .b()
        .onTrue(
            new DRIVE_WITH_HEADING(
                    m_robotDrive,
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    270)
                .until(() -> Math.abs(m_driverController.getRightX()) > 0.3));

    m_driverController
        .x()
        .onTrue(
            new DRIVE_WITH_HEADING(
                    m_robotDrive,
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    90)
                .until(() -> Math.abs(m_driverController.getRightX()) > 0.3));

    m_driverController.leftBumper().onTrue(m_robotDrive.zeroGyro());

    m_driverController
        .rightTrigger(.9)
        .whileTrue(
            new DRIVE_WITH_HEADING_SUPPLIER(
                m_robotDrive,
                () ->
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                () ->
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                () ->
                    new Rotation2d(
                        m_driverController.getRightX(), -m_driverController.getRightY())));

    // Run Levetator to 20mm
    m_operatorController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_levetator.setGoal(20); // goal is in mm
                  m_levetator.enable();
                },
                m_levetator));

    // Run Levetator to 0mm
    m_operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_levetator.setGoal(0);
                  m_levetator.enable();
                },
                m_levetator));

    // Disable the arm controller when Y is pressed.
    m_driverController.y().onTrue(Commands.runOnce(m_levetator::disable));
  }

    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("4 Note Auto Near Amp");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

}
