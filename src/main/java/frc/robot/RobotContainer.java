// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auton.TemplateAuton;
import frc.robot.commands.drive.AutoAim;
import frc.robot.commands.drive.RobotGotoAngle;
import frc.robot.commands.drive.TurningMotorsTest;
import frc.robot.commands.shooter.shoot;
import frc.robot.commands.vision.DefaultLimelightPipeline;
import frc.robot.commands.vision.UpdateOdometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.FieldUtils;

import java.util.Map;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(() -> m_visionSubsystem.getRobotPosition(),
      () -> m_visionSubsystem.getTimeStampEstimator());
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(() -> m_driveSubsystem.getRobotPitch());
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ShooterPivotSubsystem m_shooterPivotSubsystem = new ShooterPivotSubsystem();
  private final ShooterIndexSubsystem m_shooterIndexSubsystem = new ShooterIndexSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakePivotSubsystem m_intakePivotSubsystem = new IntakePivotSubsystem();

  // The driver's controller
  // final Joystick m_driverController = new
  // Joystick(OIConstants.kDriverControllerPort);
  final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  final XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  SendableChooser<Command> m_autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    configureButtonBindings();

    // Configure limelight default pipeline
    m_visionSubsystem.setDefaultCommand(new DefaultLimelightPipeline(m_visionSubsystem));

    // Configure default commands
    m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_driveSubsystem.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_driveSubsystem));
    // Joystick equivalent:
    // new RunCommand(
    // () -> m_robotDrive.drive(
    // -MathUtil.applyDeadband(m_driverController.getY(),
    // OIConstants.kDriveDeadband),
    // -MathUtil.applyDeadband(m_driverController.getX(),
    // OIConstants.kDriveDeadband),
    // -MathUtil.applyDeadband(m_driverController.getTwist(),
    // OIConstants.kDriveDeadband),
    // true, true),
    // m_robotDrive));

    // "registerCommand" lets pathplanner identify our commands
    // Here's the autoalign as an example:
    // NamedCommands.registerCommand("Auto Align",
    // new AutoAim(
    // m_visionSubsystem,
    // m_robotDrive,
    // () -> m_driverController.getLeftY(),
    // () -> m_driverController.getLeftX()
    // )
    // );

    // Adding options to the sendable chooser
    m_autonChooser.setDefaultOption("Template Auton", new TemplateAuton(m_driveSubsystem));
    m_autonChooser.addOption("Path Planner", new PathPlannerAuto("Move One Meter"));
    m_autonChooser.addOption("Path Planner", new PathPlannerAuto("Two Meter Spin"));

    // Put chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add("Select Auton", m_autonChooser).withSize(2, 1);

    // DEBUG: shuffleboard widget for resetting pose. For now I'm using a default
    // pose of 0, 0 and a rotation of 0
    Shuffleboard.getTab("Swerve").add("Reset Pose", new InstantCommand(this::resetPose));

    // DEBUG: shuffleboard widget for manually setting the odometry equal to the
    // vision calculation
    Shuffleboard.getTab("Vision").add("Update Odometry", new UpdateOdometry(m_driveSubsystem, m_visionSubsystem));

    // DEBUG: widgets for testing swerve modules
    Shuffleboard.getTab("Swerve").add("Module Drive Test", new RunCommand(
        () -> m_driveSubsystem.drive(
            0.1,
            0,
            0,
            false, true),
        m_driveSubsystem));
    Shuffleboard.getTab("Swerve").add("Module Turn Test", new TurningMotorsTest(m_driveSubsystem));

    // FAILSAFE: widgets for manually setting robot position in front of notes
    Shuffleboard.getTab("Autonomous").add("Set Amp Side",
      new InstantCommand(() -> m_driveSubsystem.resetOdometry(FieldUtils.flipRed(
        new Pose2d(
          1.5, 
          7, 
          Rotation2d.fromDegrees(180))
      )))
    );

    Shuffleboard.getTab("Autonomous").add("Set Middle",
      new InstantCommand(() -> m_driveSubsystem.resetOdometry(FieldUtils.flipRed(
        new Pose2d(
          1.5, 
          5.55, 
          Rotation2d.fromDegrees(180))
      )))
    );

    Shuffleboard.getTab("Autonomous").add("Set Source Side",
      new InstantCommand(() -> m_driveSubsystem.resetOdometry(FieldUtils.flipRed(
        new Pose2d(
          1.5, 
          4.1, 
          Rotation2d.fromDegrees(180))
      )))
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Right bumper: puts drive into x mode
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_driveSubsystem.setX(),
            m_driveSubsystem));

    // Left bumper: sets gyro to 0 degrees
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(
            () -> m_driveSubsystem.zeroHeading(),
            m_driveSubsystem));

    // Y button: auto aim
    // This is commented out until the Shooter pivot subsystem works
    // new JoystickButton(m_driverController, Button.kY.value)
    // .toggleOnTrue(
    // new AutoAim(
    // m_visionSubsystem,
    // m_robotDrive,
    // () -> m_driverController.getLeftY(),
    // () -> m_driverController.getLeftX()
    // )
    // );

    // A button: makes robot face 0 degrees
    new JoystickButton(m_driverController, Button.kA.value)
        .toggleOnTrue(
            new RobotGotoAngle(
                m_driveSubsystem,
                0,
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX()));

    // Joystick equivalent:
    // new JoystickButton(m_driverController, Button.kA.value)
    // .toggleOnTrue(
    // new RobotGotoAngle(
    // m_robotDrive,
    // 0,
    // () -> m_driverController.getY(),
    // () -> m_driverController.getX()
    // )
    // );

    // B button: sets gyro to 90 degrees
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new InstantCommand(
            () -> m_driveSubsystem.setHeading(90),
            m_driveSubsystem));

    // Button for testing shooter:
    new JoystickButton(m_coDriverController, Button.kX.value)
        .onTrue(new InstantCommand(
            () -> m_shooterSubsystem.setBothSpeeds(20), m_shooterSubsystem))
        .onFalse(
            new InstantCommand(
                m_shooterSubsystem::stopRollers, m_shooterSubsystem));

    // Button for testing shooter index:
    new JoystickButton(m_coDriverController, Button.kY.value)
        .onTrue(
            new InstantCommand(
                () -> m_shooterIndexSubsystem.setSpeed(1)))
        .onFalse(
            new InstantCommand(
                () -> m_shooterIndexSubsystem.setSpeed(0)));

    // Use bumpers for intake pivot
    new JoystickButton(m_coDriverController, Button.kLeftBumper.value)
        .onTrue(
            new InstantCommand(() -> m_intakePivotSubsystem.setSpeed(-0.5)))
        .onFalse(
            new InstantCommand(() -> m_intakePivotSubsystem.setSpeed(0)));

    new JoystickButton(m_coDriverController, Button.kRightBumper.value)
        .onTrue(
            new InstantCommand(() -> m_intakePivotSubsystem.setSpeed(0.3)))
        .onFalse(
            new InstantCommand(() -> m_intakePivotSubsystem.setSpeed(0)));

    // Dpad up: shooter pivot up
    new POVButton(m_coDriverController, 0)
        .onTrue(
            new InstantCommand(() -> m_shooterPivotSubsystem.setSpeed(1)))
        .onFalse(
            new InstantCommand(() -> m_shooterPivotSubsystem.setSpeed(0)));

    // Dpad down: shooter pivot down
    new POVButton(m_coDriverController, 180)
        .onTrue(
            new InstantCommand(() -> m_shooterPivotSubsystem.setSpeed(-1)))
        .onFalse(
            new InstantCommand(() -> m_shooterPivotSubsystem.setSpeed(0)));

    // Indexer button
    // new JoystickButton(m_coDriverController, Button.kB.value)
    // .onTrue(
    // new InstantCommand(() -> m_intakePivotSubsystem.setSpeed(-0.2))
    // )
    // .onFalse(
    // new InstantCommand(() -> m_intakePivotSubsystem.setSpeed(0))
    // );

    // new JoystickButton(m_coDriverController, Button.kright)
    // .onTrue(new shoot(
    // () -> m_shooterSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem,
    // .5,
    // .5));
  }

  public void resetPose() {
    m_driveSubsystem.resetOdometry(
        new Pose2d(0, 0, new Rotation2d(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }
}
