package frc.robot.commands;
    
    import frc.robot.subsystems.IntakeSubsystem;
    import frc.robot.subsystems.ShooterPivotSubsystem;
    import edu.wpi.first.wpilibj2.command.CommandBase; 
    import frc.robot.Constants.IntakeConstants; 
    import java.util.function.DoubleSupplier;

public class intakeHumanPlayer {
    
    public class intakeHumanPlayer extends Command {
      private final IntakeSubsystem m_intakeSystem;  
      private final ShooterPivotSubsystem m_shooterPivotSubsystem; 
      private final IndexSubsystem m_indexSubsystem;
      //variables for motor speeds/velocities 

      private final DoubleSupplier topSMotorV;
      private final DoubleSupplier bottomSMotorV;
      
    
      /**
       * Creates a new shootCommand.
       *
       * @param Shootersubsystem The subsystem used by this command.
       */ 

      public intakeHumanPlayer(IntakeSubsystem intakeSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, IndexSubsystem indexSubsystem, DoubleSupplier topSMotorV, DoubleSupplier bottomSMotorV) {
        m_intakeSystem = intakeSubsystem; 
        m_shooterPivotSubsystem = shooterPivotSubsystem; 
        m_indexSubsystem = indexSubsystem;
        
        m_topSMotorV = topSMotorV; 
        m_bottomSMotorV = bottomSMotorV;  
        addRequirements(intakeSubsystem, shooterPivotSubsystem, indexSubsystem);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        m_complete = false;
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        if (m_shooterSubsystem.gamePieceDetected() == false){
          indexSubsystem.setSpeed(IntakeConstants.kHumanPlayerIntakeSpeed);


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
    
}
