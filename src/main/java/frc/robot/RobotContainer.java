// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.BottomButtons.*;
import static frc.robot.Constants.OIConstants.TopButtons.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.LeftClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RightClimberConstants;
import frc.robot.SetPoints.LevetatorSetpoints;
import frc.robot.SetPoints.PivotSetpoints;
import frc.robot.SetPoints.WristSetpoints;
import frc.robot.commands.amp.INTAKE_SHOOTER_SMART_AMP;
import frc.robot.commands.climber.CLIMBER_TO_HEIGHT;
import frc.robot.commands.drive.AMP_ALIGN_DRIVE;
import frc.robot.commands.drive.DRIVE_WITH_HEADING;
import frc.robot.commands.shooting.shooterVariableDistanceSpeedCommand;
import frc.robot.subsystems.ClimberModule;
import frc.robot.subsystems.ClimberModule.PID_Slot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LevetatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.utils.Limelight;
import frc.robot.subsystems.utils.LimelightPipeline;
import java.util.Map;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer {

  // The robot's subsystems
  private final Limelight m_frontLimelight = new Limelight("limelight-front");
  private final Limelight m_rearLimelight = new Limelight("limelight-rear");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final LevetatorSubsystem m_levetator = new LevetatorSubsystem();
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

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandGenericHID m_operatorButtonsTop =
      new CommandGenericHID(OIConstants.kOperatorControllerPort);
  CommandGenericHID m_operatorButtonsBottom =
      new CommandGenericHID(OIConstants.kOperatorController2Port);

  public AmpDirection AMP_DIRECTION = AmpDirection.REAR;

  private final SendableChooser<Command> autoChooser;

  public AmpDirection selectAmpDirection() {
    var ampDir = AmpDirection.REAR;
    double robotHeading = m_robotDrive.getHeading().getDegrees();
    robotHeading = MathUtil.inputModulus(robotHeading, -180, 180);
    SmartDashboard.putNumber("Mod Header", robotHeading);
    if (robotHeading > 0.0) {
      ampDir = AmpDirection.FRONT;
    } else {
      ampDir = AmpDirection.REAR;
    }
    return ampDir;
  }

  public void autoInit() {
    m_robotDrive.zeroGyro();
  }

  public Command intakePositionCommand() {
    return (m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kIntake))
        .andThen(m_levetator.levInRange().withTimeout(1))
        // .andThen(m_levetator.setSquishyModeCommand())
        .andThen(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kIntake)
                .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kIntake)));
  }

  public Command stowedCommand() {
    return m_pivot
        .pivotSetpointCommand(PivotSetpoints.kStowed)
        .alongWith(m_shooter.shooterStop().alongWith(m_intake.intakeStop()))
        .andThen(m_pivot.pivotInRange().withTimeout(1))
        .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
        .andThen(m_wrist.wristInRange().withTimeout(1))
        .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed));
  }

  public Command aimCommand() {
    return new RunCommand(
            () ->
                m_robotDrive.drive(0, 0, limelight_aim_proportional(m_frontLimelight), true, true),
            m_robotDrive)
        .withTimeout(.5);
  }

  public Command trapPrepCommand() {
    return m_levetator
        .levetatorSetpointPosition(LevetatorSetpoints.kStowed)
        .andThen(m_intake.moveNoteCommand().withTimeout(1))
        .andThen(m_wrist.wristAngleSetpoint(2.70).alongWith(m_pivot.pivotSetpointCommand(3.3)))
        .andThen(m_pivot.pivotInRange())
        .andThen(m_wrist.wristAngleSetpoint(3.75));
  }

  public Command trapFinishCommand() {
    return m_wrist
        .wristAngleSetpoint(3.14)
        .andThen(new WaitCommand(.5))
        .andThen(m_levetator.levetatorSetpointPosition(.1));
  }

  public Command trapExtensionCommand() {
    return m_pivot
        .pivotSetpointCommand(3.05).alongWith(m_wrist.wristAngleSetpoint(3.49))
        .andThen(m_pivot.pivotInRange())
        .andThen(m_wrist.wristAngleSetpoint(3.49))
        .alongWith(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kTrap))
        .andThen(m_pivot.pivotSetpointCommand(3.12))
        .andThen(m_wrist.wristAngleSetpoint(3.51));
  }

    public Command trapExtensionV2Command() {
        return 
            m_levetator.levetatorSetpointPosition(0.030)
            .andThen(new WaitCommand(.1))
            .andThen(m_pivot.pivotSetpointCommand(3))
            .andThen(m_pivot.pivotInRange().withTimeout(.3))
            .andThen(m_wrist.wristAngleSetpoint(3.49))
            .alongWith(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kTrap))
            .andThen(m_pivot.pivotSetpointCommand(3.16))
            .andThen(m_wrist.wristAngleSetpoint(3.49));
    } 

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

  public Command harmonyClimbExtendCommand() {
    return new CLIMBER_TO_HEIGHT(m_leftClimber, m_rightClimber, Units.inchesToMeters(18), false)
        .andThen(m_pivot.pivotSetpointCommand(2.2));
  }

  public Command rumbleControllerStop() {
    return Commands.runOnce(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0));
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand(
        "Pos1Shot",
        m_pivot
            .pivotSetpointCommand(PivotSetpoints.kVariableShot)
            .andThen(m_wrist.wristAngleSetpoint(2.6).alongWith(m_intake.intakePrepShoot()))
            .andThen(m_shooter.shooterSpeakerShot())
            .andThen(new WaitCommand(.2))
            .andThen(m_intake.intakeShootCommand().withTimeout(1))
            .andThen(m_shooter.shooterStop()));
    NamedCommands.registerCommand("setX", m_robotDrive.setXCommand());
    NamedCommands.registerCommand(
        "Shoot",
        m_levetator
            .levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer)
            .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kVariableShot))
            .andThen(m_intake.intakePrepShoot().andThen(m_shooter.shooterSpeakerShot()))
            .andThen(
                new shooterVariableDistanceSpeedCommand(
                        m_shooter, m_wrist, m_frontLimelight::SimpleDistanceToSpeakerMeters)
                    .withTimeout(.5))
            .andThen(m_wrist.wristInRange().withTimeout(1))
            .andThen(m_intake.intakeShootCommand().withTimeout(1))
            .andThen(m_shooter.shooterStop()));

    NamedCommands.registerCommand(
        "Starting Shoot",
        m_frontLimelight
            .setPipelineCommand(LimelightPipeline.SHOOT)
            .andThen(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer))
            .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kVariableShot))
            .andThen(m_intake.intakePrepShoot().andThen(m_shooter.shooterSpeakerShot()))
            .andThen(
                new shooterVariableDistanceSpeedCommand(
                        m_shooter, m_wrist, m_frontLimelight::SimpleDistanceToSpeakerMeters)
                    .withTimeout(.5))
            .andThen(m_wrist.wristInRange().withTimeout(1))
            .andThen(m_intake.intakeShootCommand().withTimeout(1))
            .andThen(m_shooter.shooterStop()));

    NamedCommands.registerCommand(
        "Shoot 1",
        m_levetator
            .levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer)
            .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kVariableShot))
            .andThen(new WaitCommand(.6))
            .andThen(m_intake.intakePrepShoot().andThen(m_shooter.shooterSpeakerShot()))
            .andThen(m_wrist.wristAngleSetpoint(2.89).andThen(m_wrist.wristInRange()))
            .andThen(new WaitCommand(.4))
            .andThen(m_intake.intakeShootCommand().withTimeout(1))
            .andThen(m_shooter.shooterStop()));

    NamedCommands.registerCommand(
        "Shoot 2",
        m_levetator
            .levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer)
            .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kVariableShot))
            .andThen(m_intake.intakePrepShoot().andThen(m_shooter.shooterSpeakerShot()))
            .andThen(new WaitCommand(.2))
            .andThen(m_wrist.wristAngleSetpoint(3.28).andThen(m_wrist.wristInRange()))
            .andThen(m_intake.intakeShootCommand().withTimeout(1))
            .andThen(m_shooter.shooterStop()));

    NamedCommands.registerCommand(
        "Shoot 3",
        m_levetator
            .levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer)
            .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kVariableShot))
            .andThen(m_intake.intakePrepShoot().andThen(m_shooter.shooterSpeakerShot()))
            .andThen(m_wrist.wristAngleSetpoint(3.325).andThen(m_wrist.wristInRange()))
            .andThen(m_intake.intakeShootCommand().withTimeout(1))
            .andThen(m_shooter.shooterStop()));

    NamedCommands.registerCommand(
        "Shoot 4",
        m_levetator
            .levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer)
            .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kVariableShot))
            .andThen(m_intake.intakePrepShoot().andThen(m_shooter.shooterSpeakerShot()))
            .andThen(m_wrist.wristAngleSetpoint(3.4).andThen(m_wrist.wristInRange()))
            .andThen(m_intake.intakeShootCommand().withTimeout(1))
            .andThen(m_shooter.shooterStop()));

    NamedCommands.registerCommand("ChaosOn", m_intake.intakeIntake().alongWith(m_shooter.shooterChaos()));
    NamedCommands.registerCommand("ShooterOff", m_shooter.shooterStop());
    NamedCommands.registerCommand("Intake Position", intakePositionCommand());
    NamedCommands.registerCommand("Intake", m_intake.intakeAutoIntake());
    NamedCommands.registerCommand("Aim", aimCommand());
    NamedCommands.registerCommand("Stowed", stowedCommand());

    configureButtonBindings();

    m_leftClimber.setGains(
        PID_Slot.CLIMBING,
        LeftClimberConstants.kP_Climbing,
        LeftClimberConstants.kI_Climbing,
        LeftClimberConstants.kD_Climbing,
        0);
    m_leftClimber.setGains(
        PID_Slot.NO_LOAD,
        LeftClimberConstants.kP_No_Climbing,
        LeftClimberConstants.kI_No_Climbing,
        LeftClimberConstants.kD_No_Climbing,
        0);
    m_rightClimber.setGains(
        PID_Slot.CLIMBING,
        RightClimberConstants.kP_Climbing,
        RightClimberConstants.kI_Climbing,
        RightClimberConstants.kD_Climbing,
        0);
    m_rightClimber.setGains(
        PID_Slot.NO_LOAD,
        RightClimberConstants.kP_No_Climbing,
        RightClimberConstants.kI_No_Climbing,
        RightClimberConstants.kD_No_Climbing,
        0);

    m_robotDrive.setDefaultCommand(
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

    m_frontLimelight.setPipelineCommand(LimelightPipeline.SHOOT);
    SmartDashboard.putData("Auto Mode", autoChooser);
    // Monologue.setupMonologue(this, "Robot", false, false);
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    SmartDashboard.putData("Zero Levetator", m_levetator.zeroLevetatorCommand());

    SmartDashboard.putData("Harmony Climb Extend", harmonyClimbExtendCommand());
    SmartDashboard.putData(
        "Harmony Climb Retract", new CLIMBER_TO_HEIGHT(m_leftClimber, m_rightClimber, 0, true));
  }

  private void configureButtonBindings() {
    /*********************/
    /*  DRIVER CONTROLS  */
    /*********************/

    m_frontLimelight.shooterTargetInRange.onTrue(
        rumbleController(.5).andThen(rumbleControllerStop()));

    // Intake
    m_driverController
        .rightTrigger()
        .whileTrue(
            m_frontLimelight
                .setPipelineCommand(LimelightPipeline.SHOOT)
                .andThen(
                    (m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kIntake))
                        .andThen(m_levetator.levInRange().withTimeout(.3)))
                // .andThen(m_levetator.setSquishyModeCommand().withTimeout(.1))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kIntake))
                .andThen(new WaitCommand(.1))
                .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kIntake))
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
        .leftTrigger()
        .and(m_operatorButtonsBottom.button(SUBWOOFER_ON).negate())
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

    /* SUBWOOFER OVERRIDE */

    m_driverController
        .leftTrigger()
        .and(m_operatorButtonsBottom.button(SUBWOOFER_ON))
        .whileTrue(
            m_levetator
                .levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer)
                .alongWith(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kVariableShot)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kSubwoofer)))
                .alongWith(m_shooter.shooterSpeakerShot()));

    m_driverController
        .leftTrigger()
        .and(m_driverController.a())
        .and(m_operatorButtonsBottom.button(SUBWOOFER_ON))
        .onTrue(m_intake.intakeShootCommand());

    // Zero IMU heading

    m_driverController.back().onTrue(m_robotDrive.zeroGyro());

    /* SMART AMP */
    // m_driverController.leftBumper().whileTrue(
    //     m_levetator.setSetpoint(LevetatorSetpoints.kAmpFront)
    //         .alongWith(m_intake.moveNoteCommand()).alongWith(m_shooter.shooterStop()));

    m_driverController
        .leftBumper()
        .whileTrue(
            m_levetator
                .levetatorSetpointPosition(LevetatorSetpoints.kAmpFront)
                .alongWith(m_intake.moveNoteCommand())
                .alongWith(m_shooter.shooterStop())
                .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kAmpFront))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpFront)));

    m_driverController
        .leftBumper()
        .and(m_driverController.a())
        .onTrue(new INTAKE_SHOOTER_SMART_AMP(m_intake, m_shooter, this::ampForward));

    m_driverController
        .rightBumper()
        .whileTrue(
            m_levetator
                .levetatorSetpointPosition(LevetatorSetpoints.kAmpRear)
                .alongWith(m_intake.moveNoteCommand())
                .alongWith(m_shooter.shooterStop())
                .andThen(m_pivot.pivotSetpointCommand(PivotSetpoints.kAmpRear))
                .andThen(m_wrist.wristAngleSetpoint(WristSetpoints.kAmpRear)));

    m_driverController
        .rightBumper()
        .and(m_driverController.a())
        .onTrue(new INTAKE_SHOOTER_SMART_AMP(m_intake, m_shooter, this::ampRear));

    m_driverController
        .leftBumper()
        .or(m_driverController.rightBumper())
        .onFalse(
            m_pivot
                .pivotSetpointCommand(PivotSetpoints.kStowed)
                .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
                .alongWith(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    // m_driverController
    //     .leftBumper()
    //     .whileTrue(
    //         (new LEVETATOR_SMART_AMP(m_levetator, this::selectAmpDirection)
    //                 .alongWith(m_intake.moveNoteCommand().alongWith(m_shooter.shooterStop()))
    //                 .andThen(
    //                     new PIVOT_SMART_AMP(m_pivot, this::selectAmpDirection)
    //                         .alongWith(new WRIST_SMART_AMP(m_wrist, this::selectAmpDirection))))
    //             .alongWith(m_ampScoringSelectV3Command));

    // m_driverController
    //     .leftBumper()
    //     .and(m_driverController.a())
    //     .onTrue(new INTAKE_SHOOTER_SMART_AMP(m_intake, m_shooter, this::selectAmpDirection));

    // m_driverController
    //     .leftBumper()
    //     .onFalse(
    //         m_pivot
    //             .pivotSetpointCommand(PivotSetpoints.kStowed)
    //             .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kStowed))
    //             .alongWith(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kStowed)));

    /* CLIMB ALIGNMENT */

    // m_driverController
    //     .rightBumper()
    //     .whileTrue(
    //         m_frontLimelight
    //             .setPipelineCommand(LimelightPipeline.STAGE)
    //             .andThen(
    //                 new STAGE_ALIGN_DRIVE(
    //                         m_robotDrive,
    //                         () ->
    //                             -MathUtil.applyDeadband(
    //                                 m_driverController.getLeftY() / 4,
    // OIConstants.kDriveDeadband),
    //                         () ->
    //                             -MathUtil.applyDeadband(
    //                                 m_driverController.getLeftX() / 4,
    // OIConstants.kDriveDeadband),
    //                         m_frontLimelight)
    //                     .andThen(
    //                         new RunCommand(
    //                             () ->
    //                                 m_robotDrive.drive(
    //                                     -MathUtil.applyDeadband(
    //                                         m_driverController.getLeftY() / 4,
    //                                         OIConstants.kDriveDeadband),
    //                                     0,
    //                                     0,
    //                                     false,
    //                                     true)))
    //                     .alongWith(
    //                         new CLIMBER_TO_HEIGHT(
    //                             m_leftClimber, m_rightClimber, Units.inchesToMeters(24),
    // false))));

    /*********************/
    /* OPERATOR CONTROLS */
    /*********************/

    /* SHOOTER SPIN UP */

    m_operatorButtonsBottom
        .button(SHOOTER_ON)
        .onTrue(m_intake.intakePrepShoot().andThen(new WaitCommand(.2)).andThen(m_shooter.shooterSpeakerShot()));
    m_operatorButtonsBottom.button(SHOOTER_OFF).onTrue(m_shooter.shooterStop());

    m_operatorButtonsBottom.button(OUTTAKE).onTrue(m_intake.intakeOuttake().withTimeout(.5));
    m_operatorButtonsBottom.button(OUTTAKE).onFalse(m_intake.intakeStop());

    m_operatorButtonsTop
        .button(CLIMB_UP)
        .onTrue(
            new CLIMBER_TO_HEIGHT(m_leftClimber, m_rightClimber, Units.inchesToMeters(24), false));

    m_operatorButtonsTop
        .button(CLIMB_DOWN)
        .onTrue(
            new CLIMBER_TO_HEIGHT(m_leftClimber, m_rightClimber, Units.inchesToMeters(1), false));

    m_operatorButtonsTop
        .button(CLIMB_TRAP)
        .onTrue(
            new CLIMBER_TO_HEIGHT(m_leftClimber, m_rightClimber, Units.inchesToMeters(-8), true));

    m_operatorButtonsTop.button(TRAP_PREP).onTrue(trapPrepCommand());

    m_operatorButtonsTop.button(CLIMB_HARM).onTrue(trapFinishCommand());

    m_operatorButtonsTop.button(TRAP_EXT).onTrue(trapExtensionCommand());

    m_operatorButtonsTop.button(AUTO_TRAP).onTrue(harmonyClimbExtendCommand());
    // m_operatorButtonsTop
    //     .button(AUTO_TRAP)
    //     .onTrue(
    //         trapPrepCommand()
    //             .andThen(new WaitCommand(.5))
    //             .andThen(trapExtensionCommand())
    //             .andThen(new WaitCommand(.5))
    //             .andThen(new CLIMBER_TO_HEIGHT(m_leftClimber, m_rightClimber, 0, true)));

    m_operatorButtonsTop
        .button(TRAP_OUTTAKE)
        .onTrue(
            (m_intake
                    .intakePrepTrap()
                    .alongWith((m_shooter.shooterTrapCommand()).andThen(new WaitCommand(.2))))
                .andThen(m_intake.intakeTransferFwd()));

    m_operatorButtonsTop
        .button(TRAP_OUTTAKE)
        .onFalse(m_shooter.shooterStop().alongWith(m_intake.intakeStop()));
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
    double kP = .01;
    double targetingAngularVelocity = ll.x() * kP;

    targetingAngularVelocity *= -1.0;

    var tv = ll.tv();
    var ampDir = selectAmpDirection();
    var gyroDeg = m_robotDrive.getHeading().getDegrees();

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

  public AmpDirection ampRear() {
    return AmpDirection.REAR;
  }

  public AmpDirection ampForward() {
    return AmpDirection.FRONT;
  }
}
