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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.LeftClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathFollowingConstraints;
import frc.robot.Constants.RightClimberConstants;
// import frc.robot.SetPoints.ClimberSetpoints;
import frc.robot.SetPoints.LevetatorSetpoints;
import frc.robot.SetPoints.PivotSetpoints;
import frc.robot.SetPoints.WristSetpoints;
// import frc.robot.Constants.GlobalConstants.StageLoc;
import frc.robot.commands.drive.DRIVE_WITH_HEADING;
import frc.robot.subsystems.ClimberModule;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LevetatorSubsystem;
// import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.utils.Limelight;
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

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    // StageChooser = new SendableChooser<>();
    // StageChooser.setDefaultOption("Stage Right", StageLoc.STAGE_RIGHT);
    // StageChooser.addOption("Stage Left", StageLoc.STAGE_LEFT);
    // StageChooser.addOption("Center Stage", StageLoc.CENTER_STAGE);

    // SmartDashboard.putData("Stage Selector", StageChooser);
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureTestButtonBindings() {
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

    /* DRIVER CONTROLS */

        m_driverController
        .y()
        .onTrue(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed)
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange().withTimeout(1))
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // Intake
    m_driverController
        .rightTrigger()
        .onTrue(
            (m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kIntake))
                .andThen(m_levetator.levInRange())
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
        .rightTrigger()
        .onFalse(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed)
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange().withTimeout(1))
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // AMP REAR (INTAKE SIDE)
    m_driverController
        .rightBumper()
        .onTrue(
            m_levetator
                .levetatorSetpointPosition(LevetatorSetpoints.kAmpRear)
                .andThen(m_levetator.levInRange().withTimeout(1))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kAmpRear)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpRear))));
    m_driverController
        .rightBumper()
        .onFalse(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed)
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange())
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    m_driverController
        .rightBumper()
        .and(m_driverController.a())
        .onTrue(
            m_levetator
                .levetatorSetpointPosition(LevetatorSetpoints.kAmpRear)
                .andThen(m_levetator.levInRange().withTimeout(1))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kAmpRear)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpRear)))
                .andThen(m_intake.intakeOuttake())
                .andThen(m_intake.intakeStop())
                .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kStowed))
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange())
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // AMP FRONT (SHOOTER)
    m_driverController
        .leftBumper()
        .onTrue(
            m_levetator
                .levetatorSetpointPosition(LevetatorSetpoints.kAmpFront)
                .andThen(m_levetator.levInRange().withTimeout(1))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kAmpFront)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpFront))));
    m_driverController
        .leftBumper()
        .onFalse(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed)
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange())
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));
//Amp Front
    m_driverController
        .leftBumper()
        .and(m_driverController.a())
        .onTrue(
            m_levetator
                .levetatorSetpointPosition(LevetatorSetpoints.kAmpFront)
                .andThen(m_levetator.levInRange().withTimeout(1))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kAmpFront)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpFront)))
                .andThen(m_shooter.shooterFeed())
                .andThen(new WaitCommand(1))
                .andThen(m_intake.intakeTransferFwd().withTimeout(1))
                .andThen(m_shooter.shooterStop().alongWith(m_intake.intakeStop()))
                .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kStowed))
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange())
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // Subwoofer shot
    m_driverController
        .leftTrigger()
        .and(m_driverController.a())
        .onTrue(
            m_shooter
                .shooterSpeakerShot()
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer))
                .andThen(m_levetator.levInRange().withTimeout(1))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kSubwoofer).withTimeout(1)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kSubwoofer).withTimeout(1)))
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
        .onTrue(
            m_shooter
                .shooterSpeakerShot()
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer))
                .andThen(m_levetator.levInRange().withTimeout(1))
                .andThen(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kSubwoofer).withTimeout(1)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kSubwoofer).withTimeout(1)))
                .andThen(new WaitCommand(2)));
                // .andThen(m_intake.intakeShootCommand())
                // .andThen(m_shooter.shooterStop())
                // .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kStowed))
                // .andThen(m_pivot.pivotInRange().withTimeout(1))
                // .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                // .andThen(m_wrist.wristInRange())
                // .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    

    m_driverController
        .leftTrigger()
        .onFalse(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed).alongWith(m_shooter.shooterStop().alongWith(m_intake.intakeStop()))
                .andThen(m_pivot.pivotInRange().withTimeout(1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .andThen(m_wrist.wristInRange().withTimeout(1))
                .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // Zero IMU heading
    m_driverController.back().onTrue(m_robotDrive.zeroGyro());

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

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
