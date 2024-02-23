// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.LeftClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathFollowingConstraints;
import frc.robot.Constants.RightClimberConstants;
// import frc.robot.SetPoints.ClimberSetpoints;
import frc.robot.SetPoints.LevetatorSetpoints;
import frc.robot.SetPoints.PivotSetpoints;
import frc.robot.SetPoints.WristSetpoints;
import frc.robot.commands.shooterVariableDistanceSpeedCommand;
// import frc.robot.Constants.GlobalConstants.StageLoc;
import frc.robot.commands.drive.DRIVE_WITH_HEADING;
// import frc.robot.Constants.GlobalConstants.StageLoc;
import frc.robot.subsystems.ClimberModule;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LevetatorSubsystem;
// import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.utils.Limelight;
import frc.robot.subsystems.utils.LimelightPipeline;
import frc.utils.ShootingCalculators;

import java.util.List;
import monologue.Logged;
import monologue.Monologue;

import java.util.Map;
import java.util.Optional;

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
                  (new RunCommand(
                      () ->
                          m_robotDrive.drive(
                              limelight_range_proportional(m_frontLimelight),
                              -MathUtil.applyDeadband(
                                  m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                              limelight_aim_proportional(m_frontLimelight),
                              false,
                              true),
                      m_robotDrive))),
              Map.entry(
                  AmpDirection.REAR,
                  (new RunCommand(
                      () ->
                          m_robotDrive.drive(
                              limelight_range_proportional(m_rearLimelight),
                              -MathUtil.applyDeadband(
                                  m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                              limelight_aim_proportional(m_rearLimelight),
                              false,
                              true),
                      m_robotDrive)))),
          this::selectAmpDirection);

    private final Command m_ampScoringSelectV2Command =
      new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(
                  AmpDirection.FRONT,
                  (new DRIVE_WITH_HEADING(
                      m_robotDrive,
                      () -> limelight_strafe_x_proportional(m_frontLimelight),
                      () -> limelight_strafe_y_proportional(m_frontLimelight),
                      90))),
              Map.entry(
                  AmpDirection.REAR,
                  (new DRIVE_WITH_HEADING(
                      m_robotDrive,
                      () -> limelight_strafe_x_proportional(m_rearLimelight),
                      () -> limelight_strafe_y_proportional(m_rearLimelight),
                      270)))),
          this::selectAmpDirection);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("setX", m_robotDrive.setXCommand());
    // NamedCommands.registerCommand(
    //     "ExtendLeftClimber_Trap", m_leftClimber.extendClimber(ClimberSetpoints.kTrapHeight));
    // NamedCommands.registerCommand(
    //     "ExtendRightClimber_Trap", m_rightClimber.extendClimber(ClimberSetpoints.kTrapHeight));

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
  }

  private void configureTestButtonBindings() {
    // m_driverController.a().onTrue(m_pivot.pivotSetpointCommand(PivotSetpoints.kAmpFront).alongWith(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));
    m_driverController
        .a()
        .onTrue(
            m_wrist
                .wristAngleSetpoint(WristSetpoints.kStowed)
                .alongWith(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));
    // m_driverController.y().onTrue(m_wrist.wristAngleSetpoint(WristSetpoints.kIntake));
    // m_driverController.x().onTrue(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed));

    m_driverController.x().onTrue(m_pivot.pivotSetpointCommand(PivotSetpoints.kIntake));
    m_driverController.y().onTrue(m_pivot.pivotSetpointCommand(PivotSetpoints.kStowed));

    /* TEST CASES */

    /* INTAKE */
    // m_operatorController
    //     .a()
    //     .onTrue(
    //         m_intake
    //             .intakeAutoIntake2()
    //             .andThen(
    //                 new RunCommand(
    //                         () ->
    //                             m_operatorController.getHID().setRumble(RumbleType.kBothRumble,
    // 1))
    //                     .withTimeout(.5)));

    /* CLIMBERS */
    // m_operatorController
    //     .povUp()
    //     .onTrue(m_rightClimber.climberForward().alongWith(m_leftClimber.climberForward()));

    // m_operatorController
    //     .povUp()
    //     .onFalse(
    //         m_rightClimber
    //             .emergencyStopClimberCommand()
    //             .alongWith(m_leftClimber.emergencyStopClimberCommand()));

    // m_operatorController
    //     .povDown()
    //     .onTrue(m_rightClimber.climberReverse().alongWith(m_leftClimber.climberReverse()));

    // m_operatorController
    //     .povDown()
    //     .onFalse(
    //         m_rightClimber
    //             .emergencyStopClimberCommand()
    //             .alongWith(m_leftClimber.emergencyStopClimberCommand()));

    /* PIVOT */
    // m_operatorController.rightBumper().onTrue(m_wrist.wristAngleSetpoint(3.14));

    // m_operatorController.a().onTrue(m_pivot.pivotSetpointCommand(2.5));
    // m_operatorController.b().onTrue(m_pivot.pivotSetpointCommand(3.14));

    // m_operatorController.y().onTrue(m_piv2.pivotTest1().andThen(m_piv2.pivotTest2()));

    /* LEVETATOR */
    // m_operatorController.x().onTrue(m_levetator.levetatorSetpointPosition(Units.inchesToMeters(2)));
    // m_operatorController
    //     .y()
    //     .onTrue(m_levetator.levetatorSetpointPosition(Units.inchesToMeters(5.5)));

    // m_operatorController.a().onTrue(m_lev.levForward());
    //     m_operatorController.a().onFalse(m_lev.levStop());
    // m_operatorController.b().onTrue(m_lev.levReverse());
    //         m_operatorController.b().onFalse(m_lev.levStop());

    /* VARIABLE SHOOTING */
    // m_operatorController
    //     .povRight()
    //     .whileTrue(
    //         m_wrist.wristAngleVariableSetpoint(
    //             () -> ShootingCalculators.DistanceToSpeakerMeters(m_robotDrive::getPose)));

    // m_operatorController
    //     .povLeft()
    //     .whileTrue(
    //         m_shooter.shooterVariableSpeakerShot(
    //             () -> ShootingCalculators.DistanceToSpeakerMeters(m_robotDrive::getPose)));

    // m_driverController
    //     .y()
    //     .onTrue(
    //         new DRIVE_WITH_HEADING_SUPPLIER(
    //                 m_robotDrive,
    //                 () ->
    //                     -MathUtil.applyDeadband(
    //                         m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //                 () ->
    //                     -MathUtil.applyDeadband(
    //                         m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //                 () -> ShootingCalculators.RotationToSpeaker(m_robotDrive::getPose))
    //             .until(() -> Math.abs(m_driverController.getRightX()) > 0.3));
    /* WRIST */

    // m_operatorController.y().onTrue(m_wrist.wristForward());
    //     m_operatorController.y().onFalse(m_wrist.wristStop());
    // m_operatorController.x().onTrue(m_wrist.wristReverse());
    //         m_operatorController.x().onFalse(m_wrist.wristStop());

    // m_operatorController.a().onTrue(m_wrist.wristTest1());
    //     // m_operatorController.a().onFalse(m_wrist.wristStop());
    // m_operatorController.b().onTrue(m_wrist.wristTest2());
    // m_operatorController.b().onFalse(m_wrist.wristStop());
    // m_operatorController.x().onTrue(m_wrist.wristStop());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

        // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", m_robotDrive.move1mYCommand());

    m_driverController
        .x()
        .onTrue(
            m_frontLimelight
                .setPipelineCommand(LimelightPipeline.SHOOT)
                .andThen(
                    new RunCommand(
                        () ->
                            m_robotDrive.drive(
                                limelight_range_proportional(m_frontLimelight),
                                -MathUtil.applyDeadband(
                                    m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                limelight_aim_proportional(m_frontLimelight),
                                false,
                                true),
                        m_robotDrive)));

    m_driverController
        .y()
        .whileTrue(
            m_frontLimelight
                .setPipelineCommand(LimelightPipeline.SHOOT)
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer).alongWith(m_pivot.pivotSetpointCommand(PivotSetpoints.kSubwoofer)))
                .andThen(new shooterVariableDistanceSpeedCommand(m_shooter, m_wrist, m_frontLimelight::SimpleDistanceToSpeakerMeters)
                        .alongWith(
                            new RunCommand(
                                () ->
                                    m_robotDrive.drive(
                                        -MathUtil.applyDeadband(
                                            m_driverController.getLeftY(),
                                            OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(
                                            m_driverController.getLeftX(),
                                            OIConstants.kDriveDeadband),
                                        limelight_aim_proportional(m_frontLimelight),
                                        true,
                                        true),
                                m_robotDrive))
                        .until(() -> Math.abs(m_driverController.getRightX()) > 0.3)));

        m_driverController
            .y().and(m_driverController.a())
            .onTrue(m_intake.intakeShootCommand());
                       
    

    m_driverController
        .y()
        .onFalse(m_frontLimelight.setPipelineCommand(LimelightPipeline.LOCALIZATION));

    m_driverController
        .b()
        .whileTrue(
            m_frontLimelight
                .setPipelineCommand(LimelightPipeline.SHOOT)
                .andThen(
                    new RunCommand(
                        () ->
                            m_robotDrive.drive(
                                -MathUtil.applyDeadband(
                                    m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(
                                    m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                limelight_aim_proportional(m_frontLimelight),
                                true,
                                true),
                        m_robotDrive)));

    m_driverController
        .x()
        .or(m_driverController.b())
        .onFalse(m_frontLimelight.setPipelineCommand(LimelightPipeline.LOCALIZATION));

    /* DRIVER CONTROLS */

    // Intake
    m_driverController
        .rightTrigger()
        .whileTrue(
            (m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kIntake))
                .andThen(m_levetator.levInRange().withTimeout(1))
                // .andThen(m_levetator.levInRange())
                // .andThen(new WaitCommand(.5))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kIntake)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kIntake)))
                .andThen(m_intake.intakeAutoIntake())
                .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kStowed))
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange().withTimeout(1))
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    m_driverController
        .rightTrigger().or(m_driverController.y())
        .onFalse(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed)
                .withTimeout(1)
                .andThen(m_shooter.shooterStop().alongWith(m_intake.intakeStop()))
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange().withTimeout(1))
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // AMP REAR (INTAKE SIDE)
    // m_driverController
    //     .rightBumper()
    //     .whileTrue(
    //         m_levetator
    //             .levetatorSetpointPosition(LevetatorSetpoints.kAmpRear)
    //             .andThen(m_levetator.levInRange().withTimeout(1))
    //             .andThen(
    //                 m_pivot
    //                     .pivotSetpointCommand(PivotSetpoints.kAmpRear)
    //                     .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpRear))));

    // m_driverController
    //     .rightBumper()
    //     .onFalse(
    //         m_pivot
    //             .pivotSetpointCommand(PivotSetpoints.kStowed)
    //             .andThen(m_pivot.pivotInRange().withTimeout(1))
    //             .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
    //             .andThen(m_wrist.wristInRange())
    //             .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // m_driverController
    //     .rightBumper()
    //     .and(m_driverController.a())
    //     .onTrue(
    //         m_levetator
    //             .levetatorSetpointPosition(LevetatorSetpoints.kAmpRear)
    //             .andThen(m_levetator.levInRange().withTimeout(1))
    //             .andThen(
    //                 m_pivot
    //                     .pivotSetpointCommand(PivotSetpoints.kAmpRear)
    //                     .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpRear)))
    //             .andThen(m_intake.intakeOuttake())
    //             .andThen(new WaitCommand(1))
    //             .andThen(m_intake.intakeStop())
    //             .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kStowed))
    //             .andThen(m_pivot.pivotInRange().withTimeout(1))
    //             .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
    //             .andThen(m_wrist.wristInRange())
    //             .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // AMP FRONT (SHOOTER)
    // m_driverController
    //     .leftBumper()
    //     .whileTrue(
    //         m_levetator
    //             .levetatorSetpointPosition(LevetatorSetpoints.kAmpFront)
    //             .andThen(m_levetator.levInRange().withTimeout(1))
    //             .andThen(
    //                 m_pivot
    //                     .pivotSetpointCommand(PivotSetpoints.kAmpFront)
    //                     .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpFront))));
    // m_driverController
    //     .leftBumper()
    //     .onFalse(
    //         m_pivot
    //             .pivotSetpointCommand(PivotSetpoints.kStowed)
    //             .andThen(m_pivot.pivotInRange().withTimeout(1))
    //             .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
    //             .andThen(m_wrist.wristInRange())
    //             .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // m_driverController
    //     .leftBumper()
    //     .and(m_driverController.a())
    //     .onTrue(
    //         m_levetator
    //             .levetatorSetpointPosition(LevetatorSetpoints.kAmpFront)
    //             .andThen(m_levetator.levInRange().withTimeout(1))
    //             .andThen(
    //                 m_pivot
    //                     .pivotSetpointCommand(PivotSetpoints.kAmpFront)
    //                     .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpFront)))
    //             .andThen(m_shooter.shooterFeed())
    //             .andThen(new WaitCommand(.5))
    //             .andThen(m_intake.intakeTransferFwd())
    //             .andThen(new WaitCommand(.5))
    //             .andThen(m_shooter.shooterStop().alongWith(m_intake.intakeStop()))
    //             .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kStowed))
    //             .andThen(m_pivot.pivotInRange().withTimeout(1))
    //             .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
    //             .andThen(m_wrist.wristInRange())
    //             .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // Subwoofer shot
    m_driverController
        .leftTrigger()
        .and(m_driverController.a())
        .whileTrue(
            m_shooter
                .shooterSpeakerShot()
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer))
                .andThen(m_levetator.levInRange().withTimeout(1))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kSubwoofer)
                        .withTimeout(1)
                        .alongWith(
                            m_wrist.wristAngleSetpoint(WristSetpoints.kSubwoofer).withTimeout(1)))
                .andThen(new WaitCommand(.5))
                .andThen(m_intake.intakeShootCommand())
                .andThen(m_shooter.shooterStop())
                .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kStowed))
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange())
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    m_driverController
        .leftTrigger()
        .whileTrue(
            m_shooter
                .shooterSpeakerShot()
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer))
                .andThen(m_levetator.levInRange().withTimeout(1))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kSubwoofer)
                        .withTimeout(1)
                        .alongWith(
                            m_wrist.wristAngleSetpoint(WristSetpoints.kSubwoofer).withTimeout(1))));

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

    m_driverController
        .povUp()
        .onTrue(m_rightClimber.climberForward().alongWith(m_leftClimber.climberForward()));
    m_driverController
        .povUp()
        .onFalse(
            m_rightClimber
                .emergencyStopClimberCommand()
                .alongWith(m_leftClimber.emergencyStopClimberCommand()));

    m_driverController
        .povDown()
        .onTrue(m_rightClimber.climberReverse().alongWith(m_leftClimber.climberReverse()));
    m_driverController
        .povDown()
        .onFalse(
            m_rightClimber
                .emergencyStopClimberCommand()
                .alongWith(m_leftClimber.emergencyStopClimberCommand()));

    /* SMART AMP */

    m_driverController
        .rightBumper()
        .whileTrue(
            m_frontLimelight
                .setPipelineCommand(LimelightPipeline.AMP)
                .alongWith(m_rearLimelight.setPipelineCommand(LimelightPipeline.AMP))
                .andThen(
                    m_levetator
                        .levetatorAmpSmartCommand(selectAmpDirection())
                        .withTimeout(1)
                        .andThen(
                            m_pivot
                                .pivotAmpSmartCommand(selectAmpDirection())
                                .alongWith(m_wrist.wristAmpSmartCommand(selectAmpDirection()))))
                .alongWith(m_ampScoringSelectCommand));

    m_driverController
        .rightBumper()
        .and(m_driverController.a())
        .whileTrue(
            m_frontLimelight
                .setPipelineCommand(LimelightPipeline.AMP)
                .alongWith(m_rearLimelight.setPipelineCommand(LimelightPipeline.AMP))
                .andThen(
                    m_levetator
                        .levetatorAmpSmartCommand(selectAmpDirection())
                        .withTimeout(1)
                        .andThen(
                            m_pivot
                                .pivotAmpSmartCommand(selectAmpDirection())
                                .alongWith(m_wrist.wristAmpSmartCommand(selectAmpDirection())))
                        .andThen(
                            m_intake
                                .intakeAmpSmartCommand(selectAmpDirection())
                                .alongWith(m_shooter.shooterAmpSmartCommand(selectAmpDirection()))
                                .andThen(new WaitCommand(1)))
                        .andThen(m_intake.intakeStop())
                        .alongWith(m_shooter.shooterStop()))
                .alongWith(m_ampScoringSelectCommand));

    m_driverController
        .rightBumper()
        .onFalse(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed)
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange())
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    /* SMART AMP OTHER ATTEMPT */

        m_driverController
            .leftBumper()
            .whileTrue(
                m_frontLimelight
                    .setPipelineCommand(LimelightPipeline.AMP)
                    .alongWith(m_rearLimelight.setPipelineCommand(LimelightPipeline.AMP))
                    .andThen(
                        m_levetator
                            .levetatorAmpSmartCommand(selectAmpDirection())
                            .withTimeout(1)
                            .andThen(
                                m_pivot
                                    .pivotAmpSmartCommand(selectAmpDirection())
                                    .alongWith(m_wrist.wristAmpSmartCommand(selectAmpDirection()))))
                    .alongWith(m_ampScoringSelectV2Command));
    
        m_driverController
            .leftBumper()
            .and(m_driverController.a())
            .whileTrue(
                m_frontLimelight
                    .setPipelineCommand(LimelightPipeline.AMP)
                    .alongWith(m_rearLimelight.setPipelineCommand(LimelightPipeline.AMP))
                    .andThen(
                        m_levetator
                            .levetatorAmpSmartCommand(selectAmpDirection())
                            .withTimeout(1)
                            .andThen(
                                m_pivot
                                    .pivotAmpSmartCommand(selectAmpDirection())
                                    .alongWith(m_wrist.wristAmpSmartCommand(selectAmpDirection())))
                            .andThen(
                                m_intake
                                    .intakeAmpSmartCommand(selectAmpDirection())
                                    .alongWith(m_shooter.shooterAmpSmartCommand(selectAmpDirection()))
                                    .andThen(new WaitCommand(1)))
                            .andThen(m_intake.intakeStop())
                            .alongWith(m_shooter.shooterStop()))
                    .alongWith(m_ampScoringSelectV2Command));
    
        m_driverController
            .leftBumper()
            .onFalse(
                m_pivot
                    .pivotSetpointCommand(PivotSetpoints.kStowed)
                    .andThen(m_pivot.pivotInRange().withTimeout(1))
                    .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                    .andThen(m_wrist.wristInRange())
                    .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));
    

    /* OPERATOR CONTROLS */

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

  public void disabledInit() {
    m_wrist.resetSetpointInit();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  double limelight_strafe_x_proportional(Limelight ll){
    double kP = .001;
    double targetXSpeed = ll.x() * kP;

    targetXSpeed *= -1.0;

    return targetXSpeed;
  }

    double limelight_strafe_y_proportional(Limelight ll){
    double kP = .001;
    double targetYSpeed = ll.y() * kP;

    targetYSpeed *= -1.0;

    return targetYSpeed;
    }

  double limelight_range_proportional(Limelight ll) {
    double kP = .1;
    double targetingForwardSpeed = ll.y() * kP;
    var tv = ll.tv();

    if (tv == 0) {
    // if no target, we want to spin in place so no forward speed
        targetingForwardSpeed = 0;
    } else {
    // invert due to limelight results
    targetingForwardSpeed *= -1.0;
    }

    return targetingForwardSpeed;
  }

  double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = m_frontLimelight.y() * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  double limelight_aim_proportional(Limelight ll) {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control
    // loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .0025;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = ll.x() * kP;

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    var tv = ll.tv();
    var ampDir = selectAmpDirection();
    var gyroDeg = m_robotDrive.getHeading();

    if (tv == 0){
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

    boolean limelight_in_range(Limelight ll) {
        var tv = ll.hasTargets();
        var ty = ll.y();
        var tx = ll.x();

        if (tv && Math.abs(ty) < 1.0 && Math.abs(tx) < 1.0) {
            return true;
        } else {
            return false;
        }
    }
}
