package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.commandGroups.shooter.LaunchNote;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.LEDSubsystem;


public class AutonShootNote extends SequentialCommandGroup{
    
    /** Command for launching a note in auton. */
    public AutonShootNote(ShooterIndexSubsystem shooterIndexSubsystem, LEDSubsystem ledSubsystem){ 
        addCommands(
            new WaitCommand(1),
            new LaunchNote(shooterIndexSubsystem, ledSubsystem)
        );
    }
}
