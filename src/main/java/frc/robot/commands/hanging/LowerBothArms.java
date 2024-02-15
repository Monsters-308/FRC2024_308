package frc.robot.commands.hanging;

import frc.robot.subsystems.HangingSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HangingConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;

//skeleton made by nico
public class LowerBothArms extends Command {
  private final HangingSubsystem m_hangingSubsystem;
  final DriveSubsystem m_driveSubsystem;
  private double hangingSpeed = .4;
  private boolean m_complete = false;
  private final PIDController pitchController = new PIDController(HangingConstants.kPitchP,
                                                              HangingConstants.kPitchI, 
                                                              HangingConstants.kPitchD);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LowerBothArms(HangingSubsystem hangingSubsystem, DriveSubsystem driveSubsystem) {
    m_hangingSubsystem = hangingSubsystem;
    m_driveSubsystem = driveSubsystem;
    
    addRequirements(hangingSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchController.reset();
    m_complete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double robotTilt = m_driveSubsystem.getGyroPitch();

    pitchController.setSetpoint(0);

    double speedDifference = pitchController.calculate(robotTilt); //speed needed to set pitch of robot with hangers

    m_hangingSubsystem.setLeftSpeed(hangingSpeed+speedDifference);
    m_hangingSubsystem.setRightSpeed(hangingSpeed-speedDifference);

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hangingSubsystem.setBothSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_complete;

  }
}
