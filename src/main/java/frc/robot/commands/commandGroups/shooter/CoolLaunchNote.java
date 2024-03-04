package frc.robot.commands.commandGroups.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.shooter.checkRPMShake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class CoolLaunchNote extends SequentialCommandGroup {

    public CoolLaunchNote(ShooterSubsystem shooterSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, LEDSubsystem ledSubsystem){
        addCommands(
            new checkRPMShake(shooterSubsystem, ShooterConstants.kWheelRevSpeed).withTimeout(5),
            new LaunchNote(shooterIndexSubsystem, ledSubsystem)
        );
    }
}
