package frc.robot.commands.commandGroups.shooter;

import frc.robot.commands.LED.setLED;
import frc.robot.commands.shooterIndex.RunShooterIndexer;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LaunchNote extends SequentialCommandGroup {

    public LaunchNote(ShooterIndexSubsystem shooterIndexSubsystem, LEDSubsystem ledSubsystem){
        addCommands(
            new setLED(ledSubsystem, ledSubsystem::purple),
            new RunShooterIndexer(shooterIndexSubsystem, 1)
                .withTimeout(0.5),
            new InstantCommand(() -> ledSubsystem.setLEDFunction(ledSubsystem::rainbow))
        );
    }
}
