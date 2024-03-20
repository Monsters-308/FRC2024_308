package frc.robot.commands.shooterIndex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterIndexConstants;
import frc.robot.subsystems.ShooterIndexSubsystem;

public class IndexNoteGood extends SequentialCommandGroup {
    
    /**
     * Runs the shooter indexer until the light sensor detects a note,
     * and then runs the indexer in reverse until the note isn't detected in order
     * to ensure that the note isn't touching the shooter wheels.
     * @param shooterIndexSubsystem
     */
    public IndexNoteGood(ShooterIndexSubsystem shooterIndexSubsystem){
        addCommands(
            // Run indexer until game piece detected
            new RunShooterIndexer(shooterIndexSubsystem, ShooterIndexConstants.kIndexIntakeSpeed)
                .until(() -> shooterIndexSubsystem.gamePieceDetected()),

            // Reverse indexer until game piece not detected
            new RunShooterIndexer(shooterIndexSubsystem, 0.8)
                .withTimeout(0.2)
        );
    }
}
