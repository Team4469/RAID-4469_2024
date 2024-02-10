// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GlobalConstants.AmpDirection;
// import frc.robot.Constants.GlobalConstants.StageLoc;
import frc.robot.Constants.LeftClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathFollowingConstraints;
import frc.robot.Constants.RightClimberConstants;
import frc.robot.SetPoints.ClimberSetpoints;
import frc.robot.commands.drive.DRIVE_WITH_HEADING;
import frc.robot.subsystems.ClimberModule;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LevetatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import java.util.Map;
import java.util.Optional;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision);
    private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final LevetatorSubsystem m_levetator = new LevetatorSubsystem(m_pivot);
  private final ClimberModule m_rightClimber =
      new ClimberModule(RightClimberConstants.kMotorID, RightClimberConstants.kSensorID, RightClimberConstants.kMotorInverted);
  private final ClimberModule m_leftClimber =
      new ClimberModule(LeftClimberConstants.kMotorID, LeftClimberConstants.kSensorID, LeftClimberConstants.kMotorInverted);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final WristSubsystem m_wrist = new WristSubsystem();
//   private final LedSubsystem m_LedSubsystem = new LedSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  private AmpDirection ampDirection = AmpDirection.REAR;

  private final SendableChooser<Command> autoChooser;

//   private final SendableChooser<StageLoc> StageChooser;

  private AmpDirection selectAmpDirection() {
    double robotHeading = m_robotDrive.getHeading();
    if (robotHeading > 0) {
      ampDirection = AmpDirection.FRONT;
      return AmpDirection.FRONT;
    } else {
      ampDirection = AmpDirection.REAR;
      return AmpDirection.REAR;
    }
  }

  private boolean allianceIsRed() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == Alliance.Red) {
      return true;
    } else return false;
  }

  //   private StageLocationAlliance selectStageLocation() {
  //     Optional<Alliance> ally = DriverStation.getAlliance();
  //     StageLocationAlliance stage;
  //     if (ally.isPresent()) {
  //         if (ally.get() == Alliance.Red) {
  //             switch (StageChooser.getSelected()) {
  //                 case STAGE_LEFT:
  //                     stage = StageLocationAlliance.STAGE_LEFT_RED;
  //                     break;
  //                 case STAGE_RIGHT:
  //                     stage = StageLocationAlliance.STAGE_RIGHT_RED;
  //                 default:
  //                     stage = StageLocationAlliance.CENTER_STAGE_RED;
  //                     break;
  //             }
  //         } else {
  //             switch (StageChooser.getSelected()) {
  //                 case STAGE_LEFT:
  //                     stage = StageLocationAlliance.STAGE_LEFT_BLUE;
  //                     break;
  //                 case STAGE_RIGHT:
  //                     stage = StageLocationAlliance.STAGE_RIGHT_BLUE;
  //                 default:
  //                     stage = StageLocationAlliance.CENTER_STAGE_BLUE;
  //                     break;
  //             }
  //         }
  //     } else {
  //         stage = StageLocationAlliance.CENTER_STAGE_BLUE;
  //     }
  //     return stage;
  //   }

  public Command m_alignSlBlueCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("SL_BLUE");
    path.preventFlipping = true;
    return AutoBuilder.pathfindThenFollowPath(path, PathFollowingConstraints.kStagePathConstraints);
  }

  public Command m_alignSrBlueCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("SR_BLUE");
    path.preventFlipping = true;
    return AutoBuilder.pathfindThenFollowPath(path, PathFollowingConstraints.kStagePathConstraints);
  }

  public Command m_alignCsBlueCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("CS_BLUE");
    path.preventFlipping = true;
    return AutoBuilder.pathfindThenFollowPath(path, PathFollowingConstraints.kStagePathConstraints);
  }

  public Command m_alignSlRedCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("SL_RED");
    path.preventFlipping = true;
    return AutoBuilder.pathfindThenFollowPath(path, PathFollowingConstraints.kStagePathConstraints);
  }

  public Command m_alignSrRedCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("SR_RED");
    path.preventFlipping = true;
    return AutoBuilder.pathfindThenFollowPath(path, PathFollowingConstraints.kStagePathConstraints);
  }

  public Command m_alignCsRedCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("CS_RED");
    path.preventFlipping = true;
    return AutoBuilder.pathfindThenFollowPath(path, PathFollowingConstraints.kStagePathConstraints);
  }

  private final ConditionalCommand m_stageRightConditional =
      new ConditionalCommand(m_alignSrRedCommand(), m_alignSrBlueCommand(), this::allianceIsRed);
  private final ConditionalCommand m_stageLeftConditional =
      new ConditionalCommand(m_alignSlRedCommand(), m_alignSlBlueCommand(), this::allianceIsRed);
  private final ConditionalCommand m_stageCenterConditional =
      new ConditionalCommand(m_alignCsRedCommand(), m_alignCsBlueCommand(), this::allianceIsRed);

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
    NamedCommands.registerCommand(
        "ExtendLeftClimber_Trap", m_leftClimber.extendClimber(ClimberSetpoints.kTrapHeight));
    NamedCommands.registerCommand(
        "ExtendRightClimber_Trap", m_rightClimber.extendClimber(ClimberSetpoints.kTrapHeight));

    SmartDashboard.putData(m_wrist);
    SmartDashboard.putData(m_levetator);
    SmartDashboard.putData(m_pivot);

    // Configure the button bindings
    configureButtonBindings();

    m_intake.setDefaultCommand(m_intake.intakeStop());

    m_shooter.setDefaultCommand(m_shooter.shooterStop());

    m_levetator.setDefaultCommand(m_levetator.positionStowed());

    m_pivot.setDefaultCommand(m_pivot.positionIntake());

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
    // StageChooser = new SendableChooser<>();
    // StageChooser.setDefaultOption("Stage Right", StageLoc.STAGE_RIGHT);
    // StageChooser.addOption("Stage Left", StageLoc.STAGE_LEFT);
    // StageChooser.addOption("Center Stage", StageLoc.CENTER_STAGE);

    // SmartDashboard.putData("Stage Selector", StageChooser);
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    /* DRIVER CONTROLS */
    // Set X
    m_driverController.rightBumper().whileTrue(m_robotDrive.setXCommand());

    // Intake
    m_driverController
        .rightTrigger()
        .onTrue(
            (m_levetator.positionIntake())
                .andThen(m_pivot.positionIntake().alongWith(m_wrist.positionIntake()))
                .andThen(m_intake.intakeAutoIntake()));

    // Rotate to amp and go to position
    m_driverController
        .a()
        .whileTrue(
            (m_ampScoringSelectCommand.alongWith(
                    (m_levetator.positionMovement())
                        .andThen(
                            m_pivot
                                .pivotAmpSmartCommand(ampDirection)
                                .alongWith(
                                    m_wrist
                                        .wristAmpSmartCommand(ampDirection)
                                        .andThen(
                                            m_levetator.levetatorAmpSmartCommand(ampDirection))))))
                .until(() -> Math.abs(m_driverController.getRightX()) > 0.3));

    // Shoot into amp
    m_driverController
        .a()
        .and(m_driverController.leftTrigger())
        .onTrue(
            m_intake
                .intakeAmpSmartCommand(ampDirection)
                .alongWith(m_shooter.shooterAmpSmartCommand(ampDirection))
                .withTimeout(1));

    // Subwoofer shot
    m_driverController
        .leftTrigger(.9)
        .onTrue(
            ((m_shooter.shooterSpeakerShot())
                    .andThen(
                        (new ParallelCommandGroup(
                            m_levetator.positionSubwoofer(),
                            m_wrist.positionSubwoofer(),
                            m_pivot.positionSubwoofer()))))
                .andThen(m_intake.intakeTransferFwd())
                .withTimeout(3));
    
    m_driverController
        .leftTrigger(.2)
        .onTrue(
            ((m_shooter.shooterSpeakerShot())
                    .andThen(
                        (new ParallelCommandGroup(
                            m_levetator.positionSubwoofer(),
                            m_wrist.positionSubwoofer(),
                            m_pivot.positionSubwoofer())))));



    // Zero IMU heading
    m_driverController.leftBumper().onTrue(m_robotDrive.zeroGyro());

    // // Use right stick as pure heading direction
    // m_driverController
    //     .rightTrigger(.9)
    //     .whileTrue(
    //         new DRIVE_WITH_HEADING_SUPPLIER(
    //             m_robotDrive,
    //             () ->
    //                 -MathUtil.applyDeadband(
    //                     m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //             () ->
    //                 -MathUtil.applyDeadband(
    //                     m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //             () ->
    //                 new Rotation2d(
    //                     m_driverController.getRightX(), -m_driverController.getRightY())));

    /* OPERATOR CONTROLS */
    // Amp Rear
    m_operatorController
        .leftBumper()
        .onTrue(
            (m_levetator.positionAmpRear())
                .andThen((m_pivot.positionAmpRear()).alongWith(m_wrist.positionAmpRear()))
                .andThen(m_intake.intakeTransferFwd().alongWith(m_shooter.shooterFeed()))
                .withTimeout(1));

    // Amp Front
    m_operatorController
        .rightBumper()
        .onTrue(
            (m_levetator.positionAmpFront())
                .andThen((m_pivot.positionAmpFront()).alongWith(m_wrist.positionAmpFront()))
                .andThen(m_intake.intakeOuttake())
                .withTimeout(1));

    // Automated Trap Sequence
    m_operatorController.y().onTrue(m_stageLeftConditional);

    m_operatorController.a().onTrue(m_stageRightConditional);

    m_operatorController.b().onTrue(m_stageCenterConditional);

    m_operatorController
        .rightTrigger()
        .and(m_operatorController.a().or(m_operatorController.b()).or(m_operatorController.y()))
        .whileTrue(
            (m_pivot.positionTrap())
                .andThen(m_wrist.positionTrap())
                .andThen(m_levetator.positionTrap())
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
        .povUp()
        .onTrue(
            m_leftClimber
                .extendClimber(ClimberSetpoints.kTrapHeight)
                .alongWith(m_rightClimber.extendClimber(ClimberSetpoints.kTrapHeight)));

    m_operatorController
        .povDown()
        .onTrue(
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
    return autoChooser.getSelected();
  }
}
