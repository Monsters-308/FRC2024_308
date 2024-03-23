package frc.robot.commands.commandGroups.shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AmpFlapSubsystem;

public class ampFlapDown extends SequentialCommandGroup  {

    /** Moves the robot in front of the amp  */
    public ampFlapDown(AmpFlapSubsystem ampFlapSubsystem){
        addCommands(
            new InstantCommand(() -> ampFlapSubsystem.setSpeed(-.1), ampFlapSubsystem),
            new WaitCommand(1),
            new InstantCommand(() -> ampFlapSubsystem.setSpeed(0), ampFlapSubsystem)
        );
    }
}
