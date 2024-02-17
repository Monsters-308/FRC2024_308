package frc.robot.commands.shooterIndex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShooterIndexSubsystem;

class launchNote extends SequentialCommandGroup {

    public launchNote(ShooterIndexSubsystem shooterIndexSubsystem){
        addCommands(
            new InstantCommand(() -> shooterIndexSubsystem.setSpeed(1), shooterIndexSubsystem),
            new WaitCommand(0.5),
            new WaitUntilCommand(() -> shooterIndexSubsystem.gamePieceDetected()).withTimeout(1),
            new InstantCommand(() -> shooterIndexSubsystem.setSpeed(0), shooterIndexSubsystem)
        );
    }
}
