package frc.robot.commands.drive;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;

class autoSpeaker extends SequentialCommandGroup  {

    public autoSpeaker(ShooterIndexSubsystem shooterIndexSubsystem, DriveSubsystem driveSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, AutoAim autoAim){
        addCommands(
            new ParallelCommandGroup(
                new RobotGotoFieldPos(driveSubsystem, 40, 10, 90), // x y and z of the postion the robot should be in to score the amp, depends on alliance??
                new InstantCommand(() -> autoAim.shooterPivotToSpeakerField())
            ));
    }
}
