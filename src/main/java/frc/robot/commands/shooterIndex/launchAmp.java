package frc.robot.commands.shooterIndex;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.RobotGotoAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drive.RobotGotoFieldPos;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShooterIndexSubsystem;

class launchAmp extends ParallelCommandGroup  {

    public launchAmp(ShooterIndexSubsystem shooterIndexSubsystem){
        addCommands(
            new RobotGotoFieldPos(null, 0, 0, 0),
            new InstantCommand(() -> shooterIndexSubsystem.setSpeed(1), shooterIndexSubsystem),
            new WaitCommand(0.5),
            new WaitUntilCommand(() -> shooterIndexSubsystem.gamePieceDetected()).withTimeout(1),
            new InstantCommand(() -> shooterIndexSubsystem.setSpeed(0), shooterIndexSubsystem)
        );
    }
}
