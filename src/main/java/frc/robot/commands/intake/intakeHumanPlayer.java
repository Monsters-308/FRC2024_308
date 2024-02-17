package frc.robot.commands.intake;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants.IntakeConstants; 
    
public class intakeHumanPlayer extends Command {
  private final IntakeSubsystem m_intakeSystem;  
  private final ShooterPivotSubsystem m_shooterPivotSubsystem; 
  private final ShooterSubsystem m_shooterSubsystem;
  private final IndexSubsystem m_indexSubsystem;
  //variables for motor speeds/velocities 

  private final double m_topSMotorV;
  private final double m_bottomSMotorV;
  

  /**
   * Creates a new shootCommand.
   *
   * @param Shootersubsystem The subsystem used by this command.
   */ 
  public intakeHumanPlayer(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterSubsystem shooterSubsystem, 
                           IndexSubsystem indexSubsystem, double topSMotorV, double bottomSMotorV) {
    m_intakeSystem = intakeSubsystem; 
    m_shooterPivotSubsystem = shooterPivotSubsystem; 
    m_indexSubsystem = indexSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    
    m_topSMotorV = topSMotorV; 
    m_bottomSMotorV = bottomSMotorV;  
    addRequirements(intakeSubsystem, shooterPivotSubsystem, indexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsystem.gamePieceDetected() == false){
      m_indexSubsystem.setSpeed(IntakeConstants.kHumanPlayerIntakeSpeed);
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
