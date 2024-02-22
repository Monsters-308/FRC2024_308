package frc.robot.commands.shooter;

import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem; 
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
  
public class shoot extends Command {  
  private final ShooterPivotSubsystem m_shooterPivotSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;    
  private final ShooterIndexSubsystem m_shooterIndexSubsystem;

  //variables for motor speeds/velocities 

  private final double m_topSMotorV;
  private final double m_bottomSMotorV;

  /**
   * Creates a new shootCommand.
   *
   * @param Shootersubsystem The subsystem used by this command.
   */ 

  public shoot(ShooterSubsystem Subsystem, ShooterPivotSubsystem pivotSubsystem, ShooterIndexSubsystem indexSubsystem , double topSMotorV, double bottomSMotorV) {
    m_shooterSubsystem = Subsystem; 
    m_shooterPivotSubsystem = pivotSubsystem; 
    m_shooterIndexSubsystem = indexSubsystem;

    
    m_topSMotorV = topSMotorV; 
    m_bottomSMotorV = bottomSMotorV;  
    addRequirements(m_shooterSubsystem, m_shooterPivotSubsystem, m_shooterIndexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterIndexSubsystem.gamePieceDetected()){

      m_shooterSubsystem.setTopShooterSpeed(m_topSMotorV); 
      m_shooterSubsystem.setBottomShooterSpeed(m_bottomSMotorV);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setTopShooterSpeed(0); 
    m_shooterSubsystem.setBottomShooterSpeed(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}