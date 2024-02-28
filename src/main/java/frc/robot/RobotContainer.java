// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.auton.TemplateAuton;
import frc.robot.commands.commandGroups.drive.autoAmp;
import frc.robot.commands.commandGroups.drive.autoSpeaker;
import frc.robot.commands.commandGroups.drive.autoTrapShoot;
import frc.robot.commands.commandGroups.intake.CompleteIntake;
import frc.robot.commands.commandGroups.shooter.LaunchNote;
import frc.robot.commands.drive.AutoAimDynamic;
import frc.robot.commands.drive.RobotGotoAngle;
import frc.robot.commands.drive.TurningMotorsTest;
import frc.robot.commands.hanging.LowerBothArms;
import frc.robot.commands.hanging.RaiseBothArms;
import frc.robot.commands.index.RunIndex;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooterIndex.IndexNoteGood;
import frc.robot.commands.shooterPivot.PivotGoToPose;
import frc.robot.commands.vision.DefaultLimelightPipeline;
import frc.robot.commands.vision.UpdateOdometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangingSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.FieldUtils;

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
  private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
  private final HangingSubsystem m_hangingSubsystem = new HangingSubsystem();


  // Controllers
  final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  final XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  SendableChooser<Command> m_autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    CameraServer.startAutomaticCapture();
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
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
                true, true),
            m_driveSubsystem));

    // "registerCommand" lets pathplanner identify our commands
    // Here's the autoalign as an example:
    NamedCommands.registerCommand("Intake Note",
      new CompleteIntake(m_intakeSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem, m_indexSubsystem, m_intakePivotSubsystem, m_LEDSubsystem)
    );

    // Adding options to the sendable chooser
    m_autonChooser.setDefaultOption("Template Auton", new TemplateAuton(m_driveSubsystem));
    m_autonChooser.addOption("One Meter", new PathPlannerAuto("Move One Meter"));
    m_autonChooser.addOption("Middle Test", new PathPlannerAuto("Simple Middle Test"));
    m_autonChooser.addOption("4 Note Auto", new PathPlannerAuto("Simple 4 note auton"));

    // Put chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add("Select Auton", m_autonChooser).withSize(2, 1);

    // DEBUG: shuffleboard widget for resetting pose. For now I'm using a default
    // pose of 0, 0 and a rotation of 0
    Shuffleboard.getTab("Swerve").add("Reset Pose", new InstantCommand(() -> m_driveSubsystem.resetOdometry(
        new Pose2d(0, 0, new Rotation2d(0)))));

    // DEBUG: shuffleboard widget for manually setting the odometry equal to the
    // vision calculation
    Shuffleboard.getTab("Vision").add("Update Odometry", new UpdateOdometry(m_driveSubsystem, m_visionSubsystem));

    // DEBUG: widgets for testing swerve modules
    Shuffleboard.getTab("Swerve").add("Module Drive Test", new RunCommand(
        () -> m_driveSubsystem.drive(
            0.03,
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
      .ignoringDisable(true)
    );

    Shuffleboard.getTab("Autonomous").add("Set Middle",
      new InstantCommand(() -> m_driveSubsystem.resetOdometry(FieldUtils.flipRed(
        new Pose2d(
          1.5, 
          5.55, 
          Rotation2d.fromDegrees(180))
      )))
      .ignoringDisable(true)
    );

    Shuffleboard.getTab("Autonomous").add("Set Source Side",
      new InstantCommand(() -> m_driveSubsystem.resetOdometry(FieldUtils.flipRed(
        new Pose2d(
          1.5, 
          4.1, 
          Rotation2d.fromDegrees(180))
      )))
      .ignoringDisable(true)
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

    //------------------------------------------- Driver buttons -------------------------------------------

    // Left bumper: sets gyro to 0 degrees
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(
            () -> m_driveSubsystem.zeroHeading()));

  // Auto Aim speaker
  new JoystickButton(m_driverController, Button.kA.value)
      .toggleOnTrue(
          // new autoSpeaker(
          //     m_driveSubsystem,
          //     m_shooterPivotSubsystem,
          //     m_shooterSubsystem
          //     ));
          new AutoAimDynamic(
            m_visionSubsystem, 
            m_driveSubsystem, 
            m_shooterPivotSubsystem,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX()));

    // // Auto Aim Amp
    // new JoystickButton(m_driverController, Button.kB.value)
    //     .toggleOnTrue(
    //         new autoAmp(
    //             m_driveSubsystem,
    //             m_shooterPivotSubsystem,
    //             m_shooterSubsystem
    //             ));
    
    // // Auto Aim trap
    // new JoystickButton(m_driverController, Button.kY.value)
    //     .toggleOnTrue(
    //         new autoTrapShoot(
    //             m_driveSubsystem,
    //             m_shooterPivotSubsystem,
    //             m_shooterSubsystem
    //             ));
    
    // Dpad up: makes robot face 0 degrees
    new POVButton(m_driverController, 0)
        .toggleOnTrue(
            new RobotGotoAngle(
                m_driveSubsystem,
                0,
                false,
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX()));

    //Dpad right: makes robot face 90 degrees to the right
    new POVButton(m_driverController, 90)
        .toggleOnTrue(
            new RobotGotoAngle(
                m_driveSubsystem,
                -90,
                false,
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX()));

    // Dpad down: makes robot face 180 degrees
    new POVButton(m_driverController, 180)
        .toggleOnTrue(
            new RobotGotoAngle(
                m_driveSubsystem,
                180,
                false,
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX()));
                
    // Dpad left: makes robot face 90 degrees to the left
    new POVButton(m_driverController, 270)
        .toggleOnTrue(
            new RobotGotoAngle(
                m_driveSubsystem,
                90,
                false,
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                () -> m_driverController.getRightX()));


    //------------------------------------------- coDriver buttons -------------------------------------------


    // // Button for testing shooter:
    new JoystickButton(m_coDriverController, Button.kX.value)
        .onTrue(new InstantCommand(
            () -> m_shooterSubsystem.setBothSpeeds(20), m_shooterSubsystem))
        .onFalse(
            new InstantCommand(
                m_shooterSubsystem::stopRollers, m_shooterSubsystem));
  
    // new POVButton(m_coDriverController, 270)
    //    .toggleOnTrue(
    //     new SequentialCommandGroup(
    //       new InstantCommand(() -> m_intakeSubsystem.setSpeed(1)),
    //       new InstantCommand(() -> m_IndexSubsystem.setSpeed(1)),
    //       new IntakeNote(m_shooterIndexSubsystem)
    //     )
    //   );

    // // Use bumpers for intake pivot
    new JoystickButton(m_coDriverController, Button.kRightBumper.value)
        .onTrue(
            new InstantCommand(() -> m_hangingSubsystem.setRightSpeed(0.8), m_hangingSubsystem))
        .onFalse(
            new InstantCommand(() -> m_hangingSubsystem.setRightSpeed(0), m_hangingSubsystem));

    new JoystickButton(m_coDriverController, Button.kLeftBumper.value)
        .onTrue(
            new InstantCommand(() -> m_hangingSubsystem.setRightSpeed(-0.8), m_hangingSubsystem))
        .onFalse(
            new InstantCommand(() -> m_hangingSubsystem.setRightSpeed(0), m_hangingSubsystem));

    //Left bumper: Raise hanging arms
    new JoystickButton(m_coDriverController, Button.kLeftBumper.value)
        .toggleOnTrue(
            new RaiseBothArms(
                m_hangingSubsystem
                ));

    // Right bumper: lower hanging arms
    new JoystickButton(m_coDriverController, Button.kRightBumper.value)
      .toggleOnTrue(
          new LowerBothArms(
              m_hangingSubsystem));
    
    // // Dpad up: intake note
    // new POVButton(m_coDriverController, 0)
    //     .toggleOnTrue(
    //         new CompleteIntake(
    //           m_intakeSubsystem,
    //           m_shooterPivotSubsystem,
    //           m_shooterIndexSubsystem,
    //           m_indexSubsystem, 
    //           m_intakePivotSubsystem
    //           ));

    // A button: aim at speaker
    new JoystickButton(m_coDriverController, Button.kA.value)
      .toggleOnTrue(
          new PivotGoToPose(
              m_shooterPivotSubsystem,
              ShooterPivotConstants.kShooterPivotSpeakerPosition
              ));

    // B button: aim at amp
    new JoystickButton(m_coDriverController, Button.kB.value)
      .toggleOnTrue(
          new PivotGoToPose(
            m_shooterPivotSubsystem,
            ShooterPivotConstants.kShooterPivotAmpPosition
              ));
    
    // Y button: aim at trap
    new JoystickButton(m_coDriverController, Button.kY.value)
    .toggleOnTrue(
      new CompleteIntake(m_intakeSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem, m_indexSubsystem, m_intakePivotSubsystem, m_LEDSubsystem)
    );

    // // Dpad up: shooter pivot up
    // new POVButton(m_coDriverController, 0)
    //     .onTrue(
    //         new InstantCommand(() -> m_shooterPivotSubsystem.setSpeed(1), m_shooterPivotSubsystem))
    //     .onFalse(
    //         new InstantCommand(() -> m_shooterPivotSubsystem.setSpeed(0), m_shooterPivotSubsystem));

    // // Dpad down: shooter pivot down
    // new POVButton(m_coDriverController, 180)
    //     .onTrue(
    //         new InstantCommand(() -> m_shooterPivotSubsystem.setSpeed(-1), m_shooterPivotSubsystem))
    //     .onFalse(
    //         new InstantCommand(() -> m_shooterPivotSubsystem.setSpeed(0), m_shooterPivotSubsystem));

    // Dpad up: Hanging arm up
    new POVButton(m_coDriverController, 0)
        .onTrue(
            new InstantCommand(() -> m_hangingSubsystem.setBothSpeed(1), m_shooterPivotSubsystem))
        .onFalse(
          new InstantCommand(() -> m_hangingSubsystem.setBothSpeed(0), m_shooterPivotSubsystem));

    // Dpad down: Hanging arm down
    new POVButton(m_coDriverController, 180)
    .onTrue(
      new InstantCommand(() -> m_hangingSubsystem.setBothSpeed(-1), m_shooterPivotSubsystem))
  .onFalse(
    new InstantCommand(() -> m_hangingSubsystem.setBothSpeed(0), m_shooterPivotSubsystem));

    // Dpad Right: intaking test
    new POVButton(m_coDriverController, 90)
        .whileTrue(
          new ParallelCommandGroup(
            new InstantCommand(() -> m_shooterPivotSubsystem.setPosition(ShooterPivotConstants.kshooterPivotDeckPosition), m_shooterPivotSubsystem),
            new InstantCommand(() -> m_intakePivotSubsystem.setPosition(IntakePivotConstants.kIntakeDownPosition), m_intakePivotSubsystem),
            new IndexNoteGood(m_shooterIndexSubsystem),
            new RunIndex(m_indexSubsystem, IndexConstants.kIndexIntakeSpeed),
            new RunIntake(m_intakeSubsystem, 1)
          )
        );

    // Right trigger: shoot note 
    new Trigger(() -> m_coDriverController.getRightTriggerAxis() > OIConstants.kTriggerDeadband)
      .toggleOnTrue(
        new LaunchNote(m_shooterIndexSubsystem).andThen(
          new InstantCommand(() -> m_shooterIndexSubsystem.setSpeed(0), m_shooterIndexSubsystem)
        )
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

