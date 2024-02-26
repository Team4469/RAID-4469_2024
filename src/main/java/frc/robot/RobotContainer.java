// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.LeftClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathFollowingConstraints;
import frc.robot.Constants.RightClimberConstants;
import frc.robot.SetPoints.LevetatorSetpoints;
import frc.robot.SetPoints.PivotSetpoints;
import frc.robot.SetPoints.WristSetpoints;
import frc.robot.commands.amp.INTAKE_SHOOTER_SMART_AMP;
import frc.robot.commands.amp.LEVETATOR_SMART_AMP;
import frc.robot.commands.amp.PIVOT_SMART_AMP;
import frc.robot.commands.amp.WRIST_SMART_AMP;
import frc.robot.commands.drive.AMP_ALIGN_DRIVE;
import frc.robot.commands.drive.DRIVE_WITH_HEADING;
import frc.robot.commands.shooterVariableDistanceSpeedCommand;
import frc.robot.subsystems.ClimberModule;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LevetatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.utils.Limelight;
import frc.robot.subsystems.utils.LimelightPipeline;
import java.util.Map;
import java.util.Optional;
import monologue.Logged;
import monologue.Monologue;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Logged {

  // The robot's subsystems
  private final Limelight m_frontLimelight = new Limelight("limelight-front");
  private final Limelight m_rearLimelight = new Limelight("limelight-rear");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_frontLimelight, m_rearLimelight);
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final LevetatorSubsystem m_levetator = new LevetatorSubsystem(m_pivot);
  private final ClimberModule m_rightClimber =
      new ClimberModule(
          RightClimberConstants.kMotorID,
          RightClimberConstants.kSensorID,
          RightClimberConstants.kMotorInverted);
  private final ClimberModule m_leftClimber =
      new ClimberModule(
          LeftClimberConstants.kMotorID,
          LeftClimberConstants.kSensorID,
          LeftClimberConstants.kMotorInverted);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final WristSubsystem m_wrist = new WristSubsystem(m_pivot);
  //   private final LedSubsystem m_LedSubsystem = new LedSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandGenericHID m_operatorController =
      new CommandGenericHID(OIConstants.kOperatorControllerPort);

  public AmpDirection AMP_DIRECTION = AmpDirection.REAR;

  private final SendableChooser<Command> autoChooser;

  //   private final SendableChooser<StageLoc> StageChooser;

  public AmpDirection selectAmpDirection() {
    var ampDir = AmpDirection.REAR;
    double robotHeading = m_robotDrive.getHeading();
    robotHeading = MathUtil.inputModulus(robotHeading, -180, 180);
    SmartDashboard.putNumber("Mod Header", robotHeading);
    if (robotHeading > 0.0) {
      ampDir = AmpDirection.FRONT;
    } else {
      ampDir = AmpDirection.REAR;
    }
    return ampDir;
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

  public Command intakePositionCommand() {
    return (m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kIntake))
        .andThen(m_levetator.levInRange().withTimeout(1))
        .andThen(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kIntake)
                .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kIntake)));
  }

  public Command stowedCommand() {
    return m_pivot
        .pivotSetpointCommand(PivotSetpoints.kStowed)
        .andThen(m_shooter.shooterStop().alongWith(m_intake.intakeStop()))
        .andThen(
            m_wrist
                .wristAngleSetpoint(WristSetpoints.kStowed)
                .alongWith(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));
  }

  public Command aimCommand() {
    return new RunCommand(
        () -> m_robotDrive.drive(0, 0, limelight_aim_proportional(m_frontLimelight), true, true),
        m_robotDrive);
  }

  private final ConditionalCommand m_stageRightConditional =
      new ConditionalCommand(m_alignSrRedCommand(), m_alignSrBlueCommand(), this::allianceIsRed);
  private final ConditionalCommand m_stageLeftConditional =
      new ConditionalCommand(m_alignSlRedCommand(), m_alignSlBlueCommand(), this::allianceIsRed);
  private final ConditionalCommand m_stageCenterConditional =
      new ConditionalCommand(m_alignCsRedCommand(), m_alignCsBlueCommand(), this::allianceIsRed);

  private final Command m_ampScoringSelectV3Command =
      new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(
                  AmpDirection.FRONT,
                  new SequentialCommandGroup(
                      m_frontLimelight.setPipelineCommand(LimelightPipeline.AMP),
                      (new AMP_ALIGN_DRIVE(
                          m_robotDrive,
                          () ->
                              -MathUtil.applyDeadband(
                                  m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                          () ->
                              -MathUtil.applyDeadband(
                                  m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                          AmpDirection.FRONT,
                          m_frontLimelight)),
                      new DRIVE_WITH_HEADING(
                          m_robotDrive,
                          this::zero,
                          () ->
                              -MathUtil.applyDeadband(
                                  m_driverController.getLeftX() / 4, OIConstants.kDriveDeadband),
                          90))),
              Map.entry(
                  AmpDirection.REAR,
                  new SequentialCommandGroup(
                      m_rearLimelight.setPipelineCommand(LimelightPipeline.AMP),
                      (new AMP_ALIGN_DRIVE(
                          m_robotDrive,
                          () ->
                              -MathUtil.applyDeadband(
                                  m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                          () ->
                              -MathUtil.applyDeadband(
                                  m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                          AmpDirection.REAR,
                          m_rearLimelight)),
                      new DRIVE_WITH_HEADING(
                          m_robotDrive,
                          this::zero,
                          () ->
                              -MathUtil.applyDeadband(
                                  m_driverController.getLeftX() / 4, OIConstants.kDriveDeadband),
                          270)))),
          this::selectAmpDirection);

  public Command rumbleController(double seconds) {
    return Commands.runOnce(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1))
        .andThen(new WaitCommand(seconds))
        .andThen(
            Commands.runOnce(
                () -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  public Command rumbleControllerStop() {
    return Commands.runOnce(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0));
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("setX", m_robotDrive.setXCommand());
    NamedCommands.registerCommand(
        "Shoot",
        m_frontLimelight
            .setPipelineCommand(LimelightPipeline.SHOOT)
            .andThen(
                m_levetator
                    .levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer)
                    .alongWith(m_pivot.pivotSetpointCommand(PivotSetpoints.kVariableShot)))
            .andThen(
                new shooterVariableDistanceSpeedCommand(
                        m_shooter, m_wrist, m_frontLimelight::SimpleDistanceToSpeakerMeters)
                    .withTimeout(.5))
            .andThen(
                new WaitCommand(.5)
                    .andThen(m_intake.intakeShootCommand().withTimeout(1))
                    .andThen(m_shooter.shooterStop())));

    NamedCommands.registerCommand("Intake Position", intakePositionCommand());
    NamedCommands.registerCommand("Intake", m_intake.intakeAutoIntake());
    NamedCommands.registerCommand("Aim", aimCommand());
    NamedCommands.registerCommand("Stowed", stowedCommand());

    // Configure the button bindings
    configureButtonBindings();
    // configureTestButtonBindings();

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

    // m_intake.setDefaultCommand(m_intake.intakeStop());

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    // StageChooser = new SendableChooser<>();
    // StageChooser.setDefaultOption("Stage Right", StageLoc.STAGE_RIGHT);
    // StageChooser.addOption("Stage Left", StageLoc.STAGE_LEFT);
    // StageChooser.addOption("Center Stage", StageLoc.CENTER_STAGE);

    // SmartDashboard.putData("Stage Selector", StageChooser);
    SmartDashboard.putData("Auto Mode", autoChooser);
    boolean fileOnly = false;
    boolean lazyLogging = false;
    Monologue.setupMonologue(this, "Robot", fileOnly, lazyLogging);
    DriverStation.startDataLog(DataLogManager.getLog(), true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /* DRIVER CONTROLS */

    m_frontLimelight.shooterTargetInRange.onTrue(
        rumbleController(.5).andThen(rumbleControllerStop()));

    // Intake
    m_driverController
        .rightTrigger()
        .whileTrue(
            (m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kIntake))
                .andThen(m_levetator.levInRange().withTimeout(1))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kIntake)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kIntake)))
                .andThen(m_intake.intakeAutoIntake())
                .andThen(
                    rumbleController(.5)
                        .alongWith(
                            m_pivot
                                .pivotSetpointCommand(PivotSetpoints.kStowed)
                                .andThen(m_pivot.pivotInRange().withTimeout(1))
                                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                                .andThen(m_wrist.wristInRange().withTimeout(1))
                                .andThen(
                                    m_levetator.levetatorSetpointPosition(
                                        LevetatorSetpoints.kStowed)))));

    m_driverController
        .rightTrigger()
        .onFalse(
            rumbleControllerStop()
                .alongWith(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kStowed)
                        .andThen(m_shooter.shooterStop().alongWith(m_intake.intakeStop()))
                        .andThen(
                            m_wrist
                                .wristAngleSetpoint(WristSetpoints.kStowed)
                                .alongWith(
                                    m_levetator.levetatorSetpointPosition(
                                        LevetatorSetpoints.kStowed)))));

    /* SHOOTING */

    m_driverController
        .rightBumper()
        .whileTrue(
            m_frontLimelight
                .setPipelineCommand(LimelightPipeline.SHOOT)
                .alongWith(
                    new RunCommand(
                        () ->
                            m_robotDrive.drive(
                                -MathUtil.applyDeadband(
                                    m_driverController.getLeftY() / 4, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(
                                    m_driverController.getLeftX() / 4, OIConstants.kDriveDeadband),
                                limelight_aim_proportional(m_frontLimelight),
                                true,
                                true),
                        m_robotDrive)));

    m_driverController
        .leftTrigger()
        .whileTrue(
            m_frontLimelight
                .setPipelineCommand(LimelightPipeline.SHOOT)
                .andThen(
                    m_levetator
                        .levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer)
                        .alongWith(m_pivot.pivotSetpointCommand(PivotSetpoints.kVariableShot)))
                .andThen(
                    new shooterVariableDistanceSpeedCommand(
                            m_shooter, m_wrist, m_frontLimelight::SimpleDistanceToSpeakerMeters)
                        .alongWith(
                            new RunCommand(
                                () ->
                                    m_robotDrive.drive(
                                        -MathUtil.applyDeadband(
                                            m_driverController.getLeftY() / 4,
                                            OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(
                                            m_driverController.getLeftX() / 4,
                                            OIConstants.kDriveDeadband),
                                        limelight_aim_proportional(m_frontLimelight),
                                        true,
                                        true),
                                m_robotDrive))
                        .until(() -> Math.abs(m_driverController.getRightX()) > 0.3)));

    m_driverController
        .leftTrigger()
        .and(m_driverController.a())
        .onTrue(m_intake.intakeShootCommand());

    m_driverController
        .leftTrigger()
        .onFalse(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed)
                .alongWith(m_shooter.shooterStop().alongWith(m_intake.intakeStop()))
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange().withTimeout(1))
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // Zero IMU heading

    m_driverController.back().onTrue(m_robotDrive.zeroGyro());

    // m_driverController
    //     .povUp()
    //     .onTrue(m_rightClimber.climberForward().alongWith(m_leftClimber.climberForward()));
    // m_driverController
    //     .povUp()
    //     .onFalse(
    //         m_rightClimber
    //             .emergencyStopClimberCommand()
    //             .alongWith(m_leftClimber.emergencyStopClimberCommand()));

    // m_driverController
    //     .povDown()
    //     .onTrue(m_rightClimber.climberReverse().alongWith(m_leftClimber.climberReverse()));
    // m_driverController
    //     .povDown()
    //     .onFalse(
    //         m_rightClimber
    //             .emergencyStopClimberCommand()
    //             .alongWith(m_leftClimber.emergencyStopClimberCommand()));
    /* SMART AMP */

    m_driverController
        .leftBumper()
        .whileTrue(
            (new LEVETATOR_SMART_AMP(m_levetator, this::selectAmpDirection)
                    .alongWith(m_intake.moveNoteCommand().alongWith(m_shooter.shooterStop()))
                    .andThen(
                        new PIVOT_SMART_AMP(m_pivot, this::selectAmpDirection)
                            .alongWith(new WRIST_SMART_AMP(m_wrist, this::selectAmpDirection))))
                .alongWith(m_ampScoringSelectV3Command));

    m_driverController
        .leftBumper()
        .and(m_driverController.a())
        .onTrue(new INTAKE_SHOOTER_SMART_AMP(m_intake, m_shooter, this::selectAmpDirection));

    m_driverController
        .leftBumper()
        .onFalse(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed)
                .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .alongWith(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    /* OPERATOR CONTROLS */

    /* CLIMBER MOVE */

    // NEEDS TO MOVE TO OPERATOR AT SOME POINT
    /* SHOOTER SPIN UP */

    m_operatorController
        .button(7)
        .onTrue(m_intake.intakePrepShoot().andThen(m_shooter.shooterSpeakerShot()));
    m_operatorController.button(8).onTrue(m_shooter.shooterStop());

    m_operatorController.button(5).onTrue(m_intake.intakeOuttake().withTimeout(.5));
    m_operatorController.button(5).onFalse(m_intake.intakeStop());

    // Automated Trap Sequence
    // m_operatorController.y().onTrue(m_stageLeftConditional);

    // m_operatorController.a().onTrue(m_stageRightConditional);

    // m_operatorController.b().onTrue(m_stageCenterConditional);

    // m_operatorController
    //     .rightTrigger()
    //     .and(m_operatorController.a().or(m_operatorController.b()).or(m_operatorController.y()))
    //     .whileTrue(
    //         (m_pivot.pivotSetpointCommand(PivotSetpoints.kTrap))
    //             .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kTrap))
    //             .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kTrap))
    //             .andThen(
    //                 m_leftClimber
    //                     .climbClimber(ClimberSetpoints.kRetractedHeight)
    //
    // .alongWith(m_rightClimber.climbClimber(ClimberSetpoints.kRetractedHeight)))
    //             .andThen(m_shooter.shooterFeed().alongWith(m_intake.intakeTransferFwd())));

    // m_operatorController
    //     .povUp()
    //     .onTrue(
    //         m_leftClimber
    //             .extendClimber(ClimberSetpoints.kTrapHeight)
    //             .alongWith(m_rightClimber.extendClimber(ClimberSetpoints.kTrapHeight)));

    // m_operatorController
    //     .povDown()
    //     .onTrue(
    //         m_leftClimber
    //             .retractClimber(ClimberSetpoints.kRetractedHeight)
    //             .alongWith(m_rightClimber.retractClimber(ClimberSetpoints.kRetractedHeight)));
  }

  public ClimberModule getLeftClimber() {
    return m_leftClimber;
  }

  public ClimberModule getRightClimber() {
    return m_rightClimber;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  double limelight_aim_proportional(Limelight ll) {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control
    // loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .01;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = ll.x() * kP;

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    var tv = ll.tv();
    var ampDir = selectAmpDirection();
    var gyroDeg = m_robotDrive.getHeading();

    if (tv == 0) {
      // if no target seen, we want to rotate in place
      if (ampDir == AmpDirection.FRONT) {
        if (gyroDeg > 90) {
          targetingAngularVelocity = -0.3;
        } else {
          targetingAngularVelocity = 0.3;
        }
      } else {
        if (gyroDeg < -90) {
          targetingAngularVelocity = -0.3;
        } else {
          targetingAngularVelocity = 0.3;
        }
      }
    }
    return targetingAngularVelocity;
  }

  public double zero() {
    return 0;
  }

  public Limelight getFrontLimelight() {
    return m_frontLimelight;
  }

  public Limelight getRearLimelight() {
    return m_rearLimelight;
  }
}
