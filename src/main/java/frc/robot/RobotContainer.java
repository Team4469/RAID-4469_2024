// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.GlobalConstants.AmpDirection;
import frc.robot.Constants.LeftClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RightClimberConstants;
import frc.robot.SetPoints.LevetatorSetpoints;
import frc.robot.SetPoints.PivotSetpoints;
import frc.robot.SetPoints.WristSetpoints;
import frc.robot.commands.amp.INTAKE_SHOOTER_SMART_AMP;
import frc.robot.commands.amp.LEVETATOR_SMART_AMP;
import frc.robot.commands.amp.PIVOT_SMART_AMP;
import frc.robot.commands.amp.WRIST_SMART_AMP;
import frc.robot.commands.climber.CLIMBER_TO_HEIGHT;
import frc.robot.commands.drive.AMP_ALIGN_DRIVE;
import frc.robot.commands.drive.DRIVE_WITH_HEADING;
import frc.robot.commands.drive.STAGE_ALIGN_DRIVE;
import frc.robot.commands.shooterVariableDistanceSpeedCommand;
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
import static frc.robot.Constants.OIConstants.BottomButtons.*;
import static frc.robot.Constants.OIConstants.TopButtons.*;
import java.util.Map;
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
  CommandGenericHID m_operatorButtonsTop =
      new CommandGenericHID(OIConstants.kOperatorControllerPort);
  CommandGenericHID m_operatorButtonsBottom =
      new CommandGenericHID(OIConstants.kOperatorController2Port);

  public AmpDirection AMP_DIRECTION = AmpDirection.REAR;

  private final SendableChooser<Command> autoChooser;

  //   private final SendableChooser<StageLoc> StageChooser;

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
        () -> m_robotDrive.drive(0, 0, limelight_aim_proportional(m_frontLimelight), true, true),
        m_robotDrive);
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
    /*********************/
    /*  DRIVER CONTROLS  */
    /*********************/

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

    /* SUBWOOFER OVERRIDE */

    m_driverController
        .leftTrigger()
        .and(m_operatorButtonsBottom.button(SUBWOOFER_ON))
        .whileTrue(
            m_levetator
                .levetatorSetpointPosition(LevetatorSetpoints.kSubwoofer)
                .alongWith(
                    m_pivot
                        .pivotSetpointCommand(PivotSetpoints.kSubwoofer)
                        .alongWith(m_wrist.wristAngleSetpoint(WristSetpoints.kSubwoofer))).alongWith(m_shooter.shooterSpeakerShot()));
    
    m_driverController
        .leftTrigger()
        .and(m_driverController.a()).and(m_operatorButtonsBottom.button(SUBWOOFER_ON))
        .onTrue(m_intake.intakeShootCommand());
    

    // Zero IMU heading

    m_driverController.back().onTrue(m_robotDrive.zeroGyro());

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

    /* CLIMB ALIGNMENT */

    m_driverController
        .rightBumper()
        .whileTrue(
            new STAGE_ALIGN_DRIVE(
                    m_robotDrive,
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftY() / 4, OIConstants.kDriveDeadband),
                    () ->
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftX() / 4, OIConstants.kDriveDeadband),
                    m_frontLimelight)
                .andThen(
                    new RunCommand(
                        () ->
                            m_robotDrive.drive(
                                -MathUtil.applyDeadband(
                                    m_driverController.getLeftY() / 4, OIConstants.kDriveDeadband),
                                0,
                                0,
                                false,
                                true)))
                .alongWith(
                    new CLIMBER_TO_HEIGHT(
                        m_leftClimber, m_rightClimber, Units.inchesToMeters(24), false)));

    /*********************/
    /* OPERATOR CONTROLS */
    /*********************/

    /* SHOOTER SPIN UP */

    m_operatorButtonsBottom
        .button(SHOOTER_ON)
        .onTrue(m_intake.intakePrepShoot().andThen(m_shooter.shooterSpeakerShot()));
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
            new CLIMBER_TO_HEIGHT(m_leftClimber, m_rightClimber, Units.inchesToMeters(0), true));

    // m_operatorButtonsTop
    //     .button(CLIMB_HARM)
    //     .onTrue(
    //         new CLIMBER_TO_HEIGHT(m_leftClimber, m_rightClimber, Units.inchesToMeters(10), true));


    m_operatorButtonsTop
        .button(TRAP_PREP)
        .onTrue(
            m_levetator
                .levetatorSetpointPosition(LevetatorSetpoints.kStowed)
                .andThen(
                    m_wrist.wristAngleSetpoint(2.70).alongWith(m_pivot.pivotSetpointCommand(3.3)))
                .andThen(m_pivot.pivotInRange())
                .andThen(m_wrist.wristAngleSetpoint(3.75)));
    // m_operatorButtonsBottom
    //     .button(9)
    //     .onTrue(
    //                 m_wrist
    //                     .wristAngleSetpoint(WristSetpoints.kIntake).andThen(new WaitCommand(1))
    //                     .alongWith(m_pivot.pivotSetpointCommand(PivotSetpoints.kIntake)));


    m_operatorButtonsTop
        .button(CLIMB_HARM)
        .onTrue(m_wrist.wristAngleSetpoint(3.14).andThen(new WaitCommand(.5)).andThen(m_levetator.levetatorSetpointPosition(.1)));
    

    m_operatorButtonsTop
        .button(TRAP_EXT)
        .onTrue(
            m_pivot
                .pivotSetpointCommand(3)
                .andThen(m_pivot.pivotInRange())
                .andThen(m_wrist.wristAngleSetpoint(3.6))
                .alongWith(m_levetator.levetatorSetpointPosition(LevetatorSetpoints.kTrap))
                .andThen(m_pivot.pivotSetpointCommand(3.14))
                .andThen(m_wrist.wristAngleSetpoint(3.67)));

    m_operatorButtonsTop
        .button(TRAP_OUTTAKE)
        .onTrue(
            m_shooter
                .shooterTrapCommand()
                .alongWith(m_intake.intakeTransferFwd()));

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
}
