package frc.robot.commands.commandGroups.shooter;

import frc.robot.subsystems.ShooterIndexSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LaunchNote extends SequentialCommandGroup {

    public LaunchNote(ShooterIndexSubsystem shooterIndexSubsystem){
        addCommands(
            // Set conveyor to max speed
            new InstantCommand(() -> shooterIndexSubsystem.setSpeed(1), shooterIndexSubsystem), 
            // Wait until note has left shooter. Use a timeout in case note is stuck or sensor is broken
            new WaitUntilCommand(() -> shooterIndexSubsystem.gamePieceDetected()).withTimeout(1), 
            // Wait a little longer just to make sure the note has fully left the shooter
            new WaitCommand(0.3),
            // Stop conveyor
            new InstantCommand(() -> shooterIndexSubsystem.setSpeed(0), shooterIndexSubsystem)
        );
    }
}
