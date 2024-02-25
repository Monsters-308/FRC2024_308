package frc.robot.commands.shooterIndex;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShooterIndexSubsystem;

public class IndexNoteGood extends SequentialCommandGroup {
    
    public IndexNoteGood(ShooterIndexSubsystem shooterIndexSubsystem){
        addCommands(
            new IndexNote(shooterIndexSubsystem),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> !shooterIndexSubsystem.gamePieceDetected()), 
                new RunShooterIndexer(shooterIndexSubsystem, -.3)
            )
        );
    }
}
