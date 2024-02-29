package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.commandGroups.shooter.LaunchNote;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonShootNote extends SequentialCommandGroup{
    
    public AutonShootNote(ShooterSubsystem shooterSubsystem, ShooterIndexSubsystem shooterIndexSubsystem){
        addCommands(
            new InstantCommand(() -> shooterSubsystem.setBothSpeeds(20.5), shooterSubsystem),
            new WaitCommand(1.5),
            new LaunchNote(shooterIndexSubsystem),
            new InstantCommand(() -> shooterSubsystem.stopRollers(), shooterSubsystem)
        );
    }
}
