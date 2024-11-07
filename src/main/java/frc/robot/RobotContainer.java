// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.AMP_ALIGN_DRIVE;
import frc.robot.commands.drive.DRIVE_WITH_HEADING;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.utils.Limelight;
import frc.robot.subsystems.utils.LimelightPipeline;
import java.util.Map;
public class RobotContainer {

  // The robot's subsystems
  private final Limelight m_frontLimelight = new Limelight("limelight-front");
  private final Limelight m_rearLimelight = new Limelight("limelight-rear");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

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


  public Command aimCommand() {
    return new RunCommand(
            () ->
                m_robotDrive.drive(0, 0, limelight_aim_proportional(m_frontLimelight), true, true),
            m_robotDrive)
        .withTimeout(.5);
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




    configureButtonBindings();

  

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
    // DriverStation.startDataLog(DataLogManager.getLog(), true);

  
  }

  private void configureButtonBindings() {
    /*********************/
    /*  DRIVER CONTROLS  */
    /*********************/

    m_frontLimelight.shooterTargetInRange.onTrue(
        rumbleController(.5).andThen(rumbleControllerStop()));




  

    // Zero IMU heading

    m_driverController.back().onTrue(m_robotDrive.zeroGyro());

    

    /*********************/
    /* OPERATOR CONTROLS */
    /*********************/



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
