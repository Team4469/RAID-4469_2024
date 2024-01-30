// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GlobalSensors;
import frc.robot.Constants.LeftClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathFollowingConstraints;
import frc.robot.Constants.RightClimberConstants;
import frc.robot.SetPoints.ClimberSetpoints;
import frc.robot.commands.drive.DRIVE_WITH_HEADING;
import frc.robot.commands.drive.DRIVE_WITH_HEADING_SUPPLIER;
import frc.robot.subsystems.ClimberModule;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LevetatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import java.util.Map;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private enum AmpDirection {
    FRONT,
    REAR
  }

  // The robot's subsystems
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision);
  private final LevetatorSubsystem m_levetator = new LevetatorSubsystem();
  private final ClimberModule m_rightClimber =
      new ClimberModule(RightClimberConstants.kMotorID, RightClimberConstants.kSensorID, false);
  private final ClimberModule m_leftClimber =
      new ClimberModule(LeftClimberConstants.kMotorID, LeftClimberConstants.kSensorID, true);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final WristSubsystem m_wrist = new WristSubsystem();
  private final LedSubsystem m_LedSubsystem = new LedSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  private AmpDirection selectAmpDirection() {
    double robotHeading = m_robotDrive.getHeading();
    if (robotHeading > 0) {
      return AmpDirection.FRONT;
    } else {
      return AmpDirection.REAR;
    }
  }

  private final Command m_ampScoringSelectCommand =
      new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(
                  AmpDirection.FRONT,
                  (new DRIVE_WITH_HEADING(
                      m_robotDrive,
                      () ->
                          -MathUtil.applyDeadband(
                              m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                      () ->
                          -MathUtil.applyDeadband(
                              m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                      90))),
              Map.entry(
                  AmpDirection.REAR,
                  (new DRIVE_WITH_HEADING(
                      m_robotDrive,
                      () ->
                          -MathUtil.applyDeadband(
                              m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                      () ->
                          -MathUtil.applyDeadband(
                              m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                      270)))),
          this::selectAmpDirection);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("setX", m_robotDrive.setXCommand());

    // Configure the button bindings
    configureButtonBindings();

    m_intake.setDefaultCommand(m_intake.intakeStop());

    m_shooter.setDefaultCommand(m_shooter.shooterStop());

    m_levetator.setDefaultCommand(m_levetator.positionStowed());

    m_pivot.setDefaultCommand(m_pivot.positionStowed());

    m_wrist.setDefaultCommand(m_wrist.positionStowed());

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

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Set X
    m_driverController.rightBumper().whileTrue(m_robotDrive.setXCommand());

    // Rotate to amp heading
    m_driverController
        .a()
        .onTrue(
            m_ampScoringSelectCommand.until(() -> Math.abs(m_driverController.getRightX()) > 0.3));

    // Zero IMU heading
    m_driverController.leftBumper().onTrue(m_robotDrive.zeroGyro());

    m_driverController
        .b()
        .onTrue(
            AutoBuilder.pathfindToPose(
                FieldPositions.StagePositions.CenterStage_Blue,
                PathFollowingConstraints.kStagePathConstraints,
                0));

    // Subwoofer shot
    m_driverController
        .leftTrigger()
        .onTrue(
            ((m_shooter.shooterSpeakerShot())
                    .andThen(
                        (new ParallelCommandGroup(
                            m_levetator.positionSubwoofer(),
                            m_wrist.positionSubwoofer(),
                            m_pivot.positionSubwoofer()))))
                .andThen(m_intake.intakeIntake())
                .withTimeout(3));

    // Use right stick as pure heading direction
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

    // Intake
    m_operatorController
        .a()
        .onTrue(
            (m_levetator.positionIntake())
                .andThen(m_pivot.positionIntake().alongWith(m_wrist.positionIntake()))
                .andThen(m_intake.intakeIntake())
                .until(m_intake.laserCanTrigger_FORWARD));

    // Amp Rear
    m_operatorController
        .b()
        .onTrue(
            (m_levetator.positionAmpRear())
                .andThen((m_pivot.positionAmpRear()).alongWith(m_wrist.positionAmpRear()))
                .andThen(m_intake.intakeTransferFwd().alongWith(m_shooter.shooterFeed()))
                .withTimeout(1));

    // Amp Front
    m_operatorController
        .x()
        .onTrue(
            (m_levetator.positionAmpFront())
                .andThen((m_pivot.positionAmpFront()).alongWith(m_wrist.positionAmpFront()))
                .andThen(m_intake.intakeOuttake())
                .withTimeout(1));

    // Automated Trap Sequence
    m_operatorController
        .y()
        .whileTrue(
            ((m_leftClimber.extendClimber(ClimberSetpoints.kTrapHeight))
                    .alongWith(m_rightClimber.extendClimber(ClimberSetpoints.kTrapHeight)))
                .andThen(
                    (m_pivot.positionTrap())
                        .andThen(m_wrist.positionTrap())
                        .andThen(m_levetator.positionTrap()))
                .andThen(
                    m_leftClimber
                        .climbClimber(ClimberSetpoints.kRetractedHeight)
                        .alongWith(m_rightClimber.climbClimber(ClimberSetpoints.kRetractedHeight)))
                .andThen(m_shooter.shooterFeed().alongWith(m_intake.intakeTransferFwd())));

    // Disable the PID controllers when stick is pressed.
    m_operatorController
        .leftStick()
        .onTrue(
            Commands.runOnce(m_levetator::disable)
                .alongWith(Commands.runOnce(m_pivot::disable), Commands.runOnce(m_wrist::disable)));

    m_operatorController
        .rightTrigger()
        .onTrue(
            m_leftClimber
                .extendClimber(ClimberSetpoints.kTrapHeight)
                .alongWith(m_rightClimber.extendClimber(ClimberSetpoints.kTrapHeight)));

    m_operatorController
        .rightTrigger()
        .onFalse(
            m_leftClimber
                .retractClimber(ClimberSetpoints.kRetractedHeight)
                .alongWith(m_rightClimber.retractClimber(ClimberSetpoints.kRetractedHeight)));
  }

  /**
   * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be called on robot
   * disable to prevent integral windup.
   */
  public void disablePIDSubsystems() {
    m_pivot.disable();
    m_levetator.disable();
    m_wrist.disable();
  }

  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("4 Note Auto Near Amp");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }
}
