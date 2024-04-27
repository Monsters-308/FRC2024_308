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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.auton.AutonShootNote;
import frc.robot.commands.auton.DynamicStartup;
import frc.robot.commands.commandGroups.intake.CompleteIntake;
import frc.robot.commands.commandGroups.intake.CompleteIntakeReverse;
import frc.robot.commands.commandGroups.shooter.LaunchNote;
import frc.robot.commands.commandGroups.shooter.ampFlapDown;
import frc.robot.commands.drive.AutoAimDynamic;
import frc.robot.commands.drive.RobotGotoAngle;
import frc.robot.commands.drive.RobotGotoFieldPos;
import frc.robot.commands.drive.TurningMotorsTest;
import frc.robot.commands.hanging.LowerBothArms;
import frc.robot.commands.hanging.RaiseBothArms;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intakePivot.SetIntakeAngle;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.CoolRevWheels;
import frc.robot.commands.shooter.RevWheels;
import frc.robot.commands.shooterIndex.IndexNote;
import frc.robot.commands.shooterPivot.DynamicPivotToSpeaker;
import frc.robot.commands.shooterPivot.PivotGoToPose;
import frc.robot.commands.vision.DefaultLimelightPipeline;
import frc.robot.subsystems.AmpFlapSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangingSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ShooterPivotSubsystem m_shooterPivotSubsystem = new ShooterPivotSubsystem();
  private final ShooterIndexSubsystem m_shooterIndexSubsystem = new ShooterIndexSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakePivotSubsystem m_intakePivotSubsystem = new IntakePivotSubsystem();
  private final HangingSubsystem m_hangingSubsystem = new HangingSubsystem();
  private final AmpFlapSubsystem m_ampFlapSubsystem = new AmpFlapSubsystem();

  // Shuffleboard toggle for field-oriented
  private final SimpleWidget m_isFieldOriented;

  // Controllers
  final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  final XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);
  // final Joystick m_buttonBox = new Joystick(OIConstants.kButtonBoxPort);

  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(() -> m_driverController.getLeftY(), () -> m_shooterIndexSubsystem.gamePieceDetected(), () -> m_visionSubsystem.getTargets());

  // Sendable chooser for launching the initial note
  SendableChooser<Command> m_autonStartup = new SendableChooser<>();

  // The second note to pick up and score
  SendableChooser<Command> m_autonFirstAction = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Put camera on shuffleboard
    CameraServer.startAutomaticCapture();

    // Configure the button bindings
    configureButtonBindings();

    // Configure limelight default pipeline
    m_visionSubsystem.setDefaultCommand(new DefaultLimelightPipeline(m_visionSubsystem));

    m_isFieldOriented = Shuffleboard.getTab("Swerve").add("Field-Oriented?", true)
      .withWidget(BuiltInWidgets.kToggleSwitch); 

    // Configure default commands
    m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_driveSubsystem.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*0.25, OIConstants.kJoystickDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*0.25, OIConstants.kJoystickDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*0.65, OIConstants.kJoystickDeadband),
                m_isFieldOriented.getEntry().getBoolean(false), true),
            m_driveSubsystem));

    // "registerCommand" lets pathplanner identify our commands

    // Complete intaking sequence
    NamedCommands.registerCommand("Complete Intake",
      new CompleteIntake(m_intakeSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem, m_intakePivotSubsystem, m_LEDSubsystem)
    );

    // First part: getting note in intake
    NamedCommands.registerCommand("Intake Note",
      new ParallelCommandGroup(
        new SetIntakeAngle(m_intakePivotSubsystem, IntakePivotConstants.kIntakeDownPosition),
        new IntakeNote(m_intakeSubsystem)
      )
    );

    // Second part: putting note in shooter (Only if there's a note in the intake)
    NamedCommands.registerCommand("Index Note",
      new SequentialCommandGroup(
        new SetIntakeAngle(m_intakePivotSubsystem, IntakePivotConstants.kIntakeDeckPosition),

        new InstantCommand(() -> m_shooterPivotSubsystem.setPosition(ShooterPivotConstants.kShooterPivotDeckPosition), m_shooterPivotSubsystem),
        new WaitUntilCommand(() -> m_shooterPivotSubsystem.inPosition()),

        new ParallelDeadlineGroup(
            new IndexNote(m_shooterIndexSubsystem), 
            new RunIntake(m_intakeSubsystem, IntakeConstants.kIntakeSpeed) 
        ),
        new InstantCommand(() -> m_shooterPivotSubsystem.setPosition(ShooterPivotConstants.kShooterPivotPoduim), m_shooterPivotSubsystem)
      ).onlyIf(() -> m_intakeSubsystem.gamePieceDetected())
    );
 
    // Launch note only if there's actually a note to shoot
    NamedCommands.registerCommand("Launch Note",
      new AutonShootNote(m_shooterIndexSubsystem, m_LEDSubsystem)
        //.onlyIf(() -> m_shooterIndexSubsystem.gamePieceDetected())
    );

    // Bring shooter pivot to speaker position
    NamedCommands.registerCommand("ShooterPivotSpeaker",
      new SequentialCommandGroup(
        new PivotGoToPose(
          m_shooterPivotSubsystem,
          ShooterPivotConstants.kShooterPivotSpeakerPosition
          ))
      );

    // Auto aim pivot to speaker
    NamedCommands.registerCommand("adjustToSpeaker",
        new DynamicPivotToSpeaker(m_shooterPivotSubsystem, m_driveSubsystem)
      );
      
    // Auto aim robot to speaker (NOTE: this uses the drive train, do not run this command while running a path)
    NamedCommands.registerCommand("faceSpeaker",
        new AutoAimDynamic(m_visionSubsystem, m_driveSubsystem, () -> 0, () -> 0, m_LEDSubsystem)
    );

    // Rev Shooter Wheels
    NamedCommands.registerCommand("Rev Shooter",
        new RevWheels(m_shooterSubsystem)
    );

    // Startup options
    // Dynamic: auto aim to goal
    m_autonStartup.setDefaultOption("Dynamic", 
      new DynamicStartup(m_shooterSubsystem, m_shooterIndexSubsystem, m_LEDSubsystem, m_shooterPivotSubsystem, m_visionSubsystem, m_driveSubsystem, m_LEDSubsystem)
    );
    
    // Static: bring shooter pivot up and launch note
    m_autonStartup.addOption("Static", 
      new SequentialCommandGroup(
        new InstantCommand(() -> m_shooterSubsystem.setPercent(1)),
        new WaitCommand(1),
        new AutonShootNote(m_shooterIndexSubsystem, m_LEDSubsystem)
      )
    );

    // Off: do nothing
    m_autonStartup.addOption("Off", new WaitCommand(0.1));
 
    // Adding options to the sendable chooser
    applyCommands(m_autonFirstAction);

    // Put choosers on the dashboard
    Shuffleboard.getTab("Autonomous").add("Launch First Note?", m_autonStartup).withSize(2, 1);
    Shuffleboard.getTab("Autonomous").add("First Action", m_autonFirstAction).withSize(2, 1);

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
          0.73, 
          6.73, 
          Rotation2d.fromDegrees(-120))
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
          0.73, 
          4.39, 
          Rotation2d.fromDegrees(120))
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

    // // Left bumper: sets gyro to 0 degrees
    // new JoystickButton(m_driverController, Button.kLeftBumper.value)
    //     .onTrue(new InstantCommand(
    //         () -> m_driveSubsystem.zeroHeading()));
    Shuffleboard.getTab("Swerve").add("Zero Gyro", new InstantCommand(
        () -> m_driveSubsystem.zeroHeading()));

    // Auto Aim speaker
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(
            new ParallelCommandGroup(
              new AutoAimDynamic(
                m_visionSubsystem, 
                m_driveSubsystem, 
                () -> m_driverController.getLeftY(),
                () -> m_driverController.getLeftX(),
                m_LEDSubsystem),

              new DynamicPivotToSpeaker(
                m_shooterPivotSubsystem, 
                m_driveSubsystem)
            )
          );

    // Auto Aim Amp
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(
            new RobotGotoFieldPos(m_driveSubsystem, FieldConstants.kAmpScoringPosition, true));
    
    // Dpad up: makes robot face 0 degrees
    new POVButton(m_driverController, 0)
    .toggleOnTrue(new RunCommand(
      () -> m_driveSubsystem.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
          m_isFieldOriented.getEntry().getBoolean(false), true),
      m_driveSubsystem));

    // Dpad right: makes robot face 90 degrees to the right
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

    // Y button: Intake note
    new JoystickButton(m_driverController, Button.kY.value)
      .toggleOnTrue(
        new CompleteIntake(m_intakeSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem, m_intakePivotSubsystem, m_LEDSubsystem)
          .andThen(
            new SetIntakeAngle(m_intakePivotSubsystem, IntakePivotConstants.kIntakeInPosition)
          ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
          .finallyDo(() -> {
            new SetIntakeAngle(m_intakePivotSubsystem, IntakePivotConstants.kIntakeInPosition).schedule();
            m_LEDSubsystem.setLEDFunction(m_LEDSubsystem::rainbow);
          })
      );
    
    // A button: fire note
    new JoystickButton(m_driverController, Button.kA.value)
      .onTrue(
          new AutoShoot(m_shooterSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem, m_LEDSubsystem, 10)
      );

    // X button: fast mode
    new JoystickButton(m_driverController, Button.kX.value)
      .onTrue(
          new AutoShoot(m_shooterSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem, m_LEDSubsystem, 30)
      );

    //------------------------------------------- coDriver buttons -------------------------------------------


    // X button: manual intake
    new JoystickButton(m_coDriverController, Button.kX.value)
        .whileTrue(
          new ParallelCommandGroup(
            new IndexNote(m_shooterIndexSubsystem),
            new RunIntake(m_intakeSubsystem, 1)
          )
        );

    // A button: aim at speaker
    new JoystickButton(m_coDriverController, Button.kA.value)
      .toggleOnTrue(
          new PivotGoToPose(
              m_shooterPivotSubsystem,
              ShooterPivotConstants.kShooterPivotSpeakerPosition
              ));

    // B button: aim at amp
    new JoystickButton(m_coDriverController, Button.kB.value)
      .onTrue(
        new SequentialCommandGroup(
          new InstantCommand(
            () -> m_shooterSubsystem.setBottomShooterSpeed(13), m_shooterSubsystem),
          new InstantCommand(
            () -> m_shooterSubsystem.setTopShooterSpeed(8), m_shooterSubsystem),
          new InstantCommand(
            () -> m_shooterPivotSubsystem.setPosition(60), m_shooterPivotSubsystem
          )
        ))
      .onFalse(
        new InstantCommand(
            m_shooterSubsystem::stopRollers, m_shooterSubsystem)
      );
    
    // Y button: Intake note
    new JoystickButton(m_coDriverController, Button.kY.value)
      .whileTrue(
        new CompleteIntake(m_intakeSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem, m_intakePivotSubsystem, m_LEDSubsystem)
          .andThen(
            new SetIntakeAngle(m_intakePivotSubsystem, IntakePivotConstants.kIntakeInPosition)
          ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
          .finallyDo(() -> {
            new SetIntakeAngle(m_intakePivotSubsystem, IntakePivotConstants.kIntakeInPosition).schedule();
            m_LEDSubsystem.setLEDFunction(m_LEDSubsystem::rainbow);
          })
      );

    // DEBUG:
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

    // Dpad up: Amp flap forwards
    new POVButton(m_coDriverController, 0)
    .whileTrue(
      new AutoShoot(m_shooterSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem, m_LEDSubsystem, 30)
      );


    // Dpad down: Amp flap backwards
    new POVButton(m_coDriverController, 180)
      .onTrue(
          new InstantCommand(() -> m_ampFlapSubsystem.setSpeed(-0.2), m_ampFlapSubsystem))
      .onFalse(
          new InstantCommand(() -> m_ampFlapSubsystem.setSpeed(0), m_ampFlapSubsystem));

    // POV left: reverse intake
    new POVButton(m_coDriverController, 270)
      .whileTrue(
          new CompleteIntakeReverse(
              m_intakeSubsystem,
              m_shooterIndexSubsystem,
              m_shooterPivotSubsystem, m_LEDSubsystem
              ));

    // Dpad right: bring intake in
    new POVButton(m_coDriverController, 90)
        .onTrue(
          new SetIntakeAngle(m_intakePivotSubsystem, IntakePivotConstants.kIntakeInPosition)
        );

    // Left bumper: Raise hanging arms
    new JoystickButton(m_coDriverController, Button.kRightBumper.value)
        .whileTrue(
            new RaiseBothArms(
                m_hangingSubsystem));

    // Right bumper: lower hanging arms
    new JoystickButton(m_coDriverController, Button.kLeftBumper.value)
        .whileTrue(
            new LowerBothArms(
                m_hangingSubsystem));

    // Right trigger: shoot note 
    new Trigger(() -> m_coDriverController.getRightTriggerAxis() > OIConstants.kTriggerDeadband)
      .toggleOnTrue(
        new LaunchNote(m_shooterIndexSubsystem, m_LEDSubsystem).andThen(
          new InstantCommand(() -> m_shooterIndexSubsystem.setSpeed(0), m_shooterIndexSubsystem)
        )
      );

    // Left trigger: charge up wheels 
    new Trigger(() -> m_coDriverController.getLeftTriggerAxis() > OIConstants.kTriggerDeadband)
      .whileTrue(
        new CoolRevWheels(m_shooterSubsystem, m_coDriverController)
      );
  }

  /**
   * Function for adding all of our auton paths to each of the choosers
   * @param autonChooser The sendable chooser being used for auton.
   */
  private void applyCommands(SendableChooser<Command> autonChooser){
    autonChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    //autonChooser.addOption("Close Note Middle", new PathPlannerAuto("AutoAim Close NOTE(CENTER SIDE)"));
    //autonChooser.addOption("Close Note Amp", new PathPlannerAuto("Close Note(AMP SIDE)"));
    //autonChooser.addOption("Close Note Source", new PathPlannerAuto("Close Note(SOURCE SIDE)"));
    //autonChooser.addOption("Far Note Middle", new PathPlannerAuto("Far Note(CENTER)"));
    //autonChooser.addOption("Far Note Amp", new PathPlannerAuto("Far Note(AMP SIDE)"));
    //autonChooser.addOption("Far Note Source", new PathPlannerAuto("Far Note(SOURCE SIDE)"));
    //autonChooser.addOption("(Test) One Meter", new PathPlannerAuto("Move One Meter"));
    //autonChooser.addOption("(Test) Middle Test", new PathPlannerAuto("Simple Middle Test"));
    //autonChooser.addOption("Close note Amp", new PathPlannerAuto("AutoAim Close NOTE(AMP SIDE)"));
    //autonChooser.addOption("source side 2 far", new PathPlannerAuto("AutoAim FAR RING(SOURCE SIDE) 2"));
    //autonChooser.addOption("source side 1 far", new PathPlannerAuto("AutoAim FAR RING(SOURCE SIDE) 1"));
    //autonChooser.addOption("source side far", new PathPlannerAuto("AutoAim FAR RING(SOURCE SIDE)"));
    //autonChooser.addOption("far ring 1", new PathPlannerAuto("AutoAim FAR RING(AMP SIDE) 1"));
    //autonChooser.addOption("far ring 2", new PathPlannerAuto("AutoAim FAR RING(AMP SIDE) 2"));
    //autonChooser.addOption("far ring 3", new PathPlannerAuto("AutoAim FAR RING(AMP SIDE) 3"));
    //autonChooser.addOption("far ring 4", new PathPlannerAuto("AutoAim FAR RING(CENTER) 4"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return new WaitCommand(15);
  }
}
