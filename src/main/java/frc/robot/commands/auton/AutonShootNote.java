package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.commandGroups.shooter.LaunchNote;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;


public class AutonShootNote extends SequentialCommandGroup{
    
    public AutonShootNote(ShooterSubsystem shooterSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, LEDSubsystem ledSubsystem){ 

        addCommands(
            new InstantCommand(() -> shooterSubsystem.setPercent(1), shooterSubsystem),
            new WaitCommand(1.7),
            new LaunchNote(shooterIndexSubsystem, ledSubsystem)

            //new InstantCommand(() -> shooterSubsystem.stopRollers(), shooterSubsystem) 
        );
    }
}
