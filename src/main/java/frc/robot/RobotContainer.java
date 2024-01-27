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
import frc.robot.commands.drive.RobotGotoAngle;
import frc.robot.commands.vision.AutoAlignAutoAim;
import frc.robot.commands.vision.AutoAlignCircle;
import frc.robot.commands.vision.DefaultLimelightPipeline;
import frc.robot.commands.vision.UpdateOdometry;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Map;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // The driver's controller
  // final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

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
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
        // new RunCommand(
        //     () -> m_robotDrive.drive(
        //         -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveDeadband),
        //         true, true),
        //     m_robotDrive));
    
    // "registerCommand" lets pathplanner identify our commands
    // Here's the autoalign as an example:
    NamedCommands.registerCommand("Auto Align", new AutoAlignAutoAim(m_visionSubsystem, m_robotDrive));

    //Adding options to the sendable chooser
    m_autonChooser.setDefaultOption("Template Auton", new TemplateAuton(m_robotDrive));
    m_autonChooser.addOption("Path Planner", new PathPlannerAuto("Move One Meter"));
    m_autonChooser.addOption("Path Planner", new PathPlannerAuto("Two Meter Spin"));

    // Put chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_autonChooser).withSize(2, 1)
      .withProperties(Map.of("Title", "Auton Command"));

    // DEBUG: shuffleboard widget for resetting pose. For now I'm using a default pose of 0, 0 and a rotation of 0
    Shuffleboard.getTab("Swerve").add("reset pose", new InstantCommand(this::resetPose)).withSize(2, 1);

    Shuffleboard.getTab("Vision").add("update odometry", new UpdateOdometry(m_robotDrive, m_visionSubsystem));
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

    //Right bumper: puts drive into x mode
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    //Left bumper: sets gyro to 0 degrees
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    
    //Y button: auto aim (high pole) (i set it to be on a button press, not held)
    new JoystickButton(m_driverController, Button.kY.value)
        .toggleOnTrue(
            new AutoAlignCircle(m_visionSubsystem, m_robotDrive)
        );
    
    //A button: makes robot face 0 degrees
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(
            new RobotGotoAngle(
              m_robotDrive,
              0,
              () -> m_driverController.getLeftY(),
              () -> m_driverController.getLeftX()
            )
        );
    // new JoystickButton(m_driverController, Button.kA.value)
    //     .toggleOnTrue(
    //         new RobotGotoAngle(
    //           m_robotDrive,
    //           0,
    //           () -> m_driverController.getY(),
    //           () -> m_driverController.getX()
    //         )
    //     );
    
    //B button: sets gyro to 90 degrees
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.setHeading(90),
            m_robotDrive));
  }

  public void resetPose(){
    m_robotDrive.resetOdometry(
        new Pose2d(0, 0, new Rotation2d(0))
    );
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
