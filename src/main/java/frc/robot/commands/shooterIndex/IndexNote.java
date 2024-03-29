package frc.robot.commands.shooterIndex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterIndexConstants;
import frc.robot.subsystems.ShooterIndexSubsystem;

public class IndexNote extends SequentialCommandGroup {
    
    /**
     * Runs the shooter indexer until the light sensor detects a note,
     * and then runs the indexer for a little longer to ensure that the 
     * note is fully in the shooter.
     * @param shooterIndexSubsystem
     */
    public IndexNote(ShooterIndexSubsystem shooterIndexSubsystem){
        addCommands(
            // Run indexer until game piece detected
            new RunShooterIndexer(shooterIndexSubsystem, ShooterIndexConstants.kIndexIntakeSpeed)
                .until(() -> shooterIndexSubsystem.gamePieceDetected()),

            // Run indexer just a little longer
            new RunShooterIndexer(shooterIndexSubsystem, 0.3)
                .withTimeout(0.05)
        );
    }
}
