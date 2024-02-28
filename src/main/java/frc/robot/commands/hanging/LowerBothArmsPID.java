package frc.robot.commands.hanging;

import frc.robot.subsystems.HangingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HangingConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;

//skeleton made by nico
public class LowerBothArmsPID extends Command {
  private final HangingSubsystem m_hangingSubsystem;
  private final DriveSubsystem m_driveSubsystem;

  private final double m_hangingSpeed;
  private boolean m_complete = false;

  private final PIDController pitchController = new PIDController(HangingConstants.kPitchP,
      HangingConstants.kPitchI,
      HangingConstants.kPitchD);

  /**
   * This is equivalent to LowerBothArms except it uses PID to keep the robot parallel to the ground.
   * We probably won't need this because we have 2 arms, but keep it in here just in case.
   * @param hangingSubsystem The hanging subsystem.
   */
  public LowerBothArmsPID(HangingSubsystem hangingSubsystem, DriveSubsystem driveSubsystem, double hangingSpeed) {
    m_hangingSubsystem = hangingSubsystem;
    m_driveSubsystem = driveSubsystem;

    m_hangingSpeed = hangingSpeed;

    // NOTE: don't add driveSubsystem because we're just using it to get data
    addRequirements(m_hangingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchController.reset();
    pitchController.setSetpoint(0);
    m_complete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double robotTilt = m_driveSubsystem.getRobotRoll();

    double speedDifference = pitchController.calculate(robotTilt); // speed needed to set roll of robot with hangers

    m_hangingSubsystem.setLeftSpeed(m_hangingSpeed + speedDifference);
    m_hangingSubsystem.setRightSpeed(m_hangingSpeed - speedDifference);

    if (m_hangingSubsystem.leftFullyRetracted() || m_hangingSubsystem.rightFullyRetracted()) {
      m_complete = true;
    }
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
