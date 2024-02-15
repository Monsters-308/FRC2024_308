package frc.robot.commands;
    
    import frc.robot.subsystems.ShooterSubsystem;
    import edu.wpi.first.wpilibj2.command.CommandBase; 
    import frc.robot.Constants.ShooterConstants; 
    import java.util.function.DoubleSupplier;

public class shoot {
    
    public class shooter extends Command {
      private final ShooterSubsystem m_shootersubsystem;  
      //variables for motor speeds/velocities 

      private final DoubleSupplier topSMotorV;
      private final DoubleSupplier bottomSMotorV;
      
    
      /**
       * Creates a new shootCommand.
       *
       * @param Shootersubsystem The subsystem used by this command.
       */ 

      public shoot(ShooterSubsystem subsystem, DoubleSupplier topSMotorV, DoubleSupplier bottomSMotorV) {
        m_shootersubsystem = subsystem; 
    
        
        m_topSMotorV = topSMotorV; 
        m_bottomSMotorV = bottomSMotorV;  
        addRequirements(Shootersubsystem);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        m_complete = false;
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        if (m_shooterSubsystem.gamePieceDetected()){

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
    
}
