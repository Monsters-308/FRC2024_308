package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.commandGroups.shooter.LaunchNote;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends SequentialCommandGroup  {

    /** Moves the robot in front of the amp  wheel speed 10*/
    public AutoShoot(ShooterSubsystem shooterSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ShooterIndexSubsystem shooterIndexSubsystem, 
                    LEDSubsystem ledSubsystem, double wheelSpeed){
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooterPivotSubsystem.setPosition(ShooterPivotConstants.kShooterPivotSpeakerPosition), shooterPivotSubsystem),
                new InstantCommand(() -> shooterSubsystem.setBothSpeeds(wheelSpeed), shooterSubsystem)
            ),
            new WaitCommand(0.7),
            new LaunchNote(shooterIndexSubsystem, ledSubsystem),
            new InstantCommand(() -> shooterSubsystem.stopRollers(), shooterSubsystem)
        );
    }
}
