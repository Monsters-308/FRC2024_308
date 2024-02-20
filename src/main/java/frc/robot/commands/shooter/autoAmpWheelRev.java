package frc.robot.commands.shooter;

import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class autoAmpWheelRev extends Command {
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  // variables for motor speeds/velocities

  public autoAmpWheelRev(ShooterPivotSubsystem pivotSubsystem, DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
    m_shooterPivotSubsystem = pivotSubsystem;
    m_driveSubsystem = driveSubsystem;
    m_shooterSubsystem = shooterSubsystem;


    addRequirements(m_shooterPivotSubsystem, m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivotSubsystem.setPosition(60);

    if(m_driveSubsystem.getPose().getX() < FieldConstants.kFieldWidthMeters/2){
      m_shooterSubsystem.setBothSpeeds(.8);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }
}
