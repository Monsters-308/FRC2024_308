package frc.robot.commands.shooter;

import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class autoWheelRevAndPivot extends Command {
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  double wheelSpeed = 0;
  double angle = 0;

  // variables for motor speeds/velocities

      /**
     * Automatically revs the shooter wheels (if robot position estimation is close to the field postion) and sets the shooter pivot
     * @param ShooterPivotSubsystem 
     * @param DriveSubsystem 
     * @param ShooterSubsystem 
     * @param WheelSpeed Sets the speed of both shooter wheels by calling m_shooterSubsystem (to use PID)
     * @param Angle Sets the angle of the shooter pivot by calling m_shooterPivotSubsystem (to use PID)
     */
  public autoWheelRevAndPivot(ShooterPivotSubsystem pivotSubsystem, DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, double WheelSpeed, double Angle) {
    m_shooterPivotSubsystem = pivotSubsystem;
    m_driveSubsystem = driveSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    wheelSpeed = WheelSpeed;
    angle = Angle; 


    addRequirements(m_shooterPivotSubsystem, m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivotSubsystem.setPosition(angle);

    if(m_driveSubsystem.getPose().getX() < FieldConstants.kFieldWidthMeters / 4){
      m_shooterSubsystem.setBothSpeeds(wheelSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopRollers();
  }
}
