package frc.robot.commands.shooterIndex;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.AutoAim;
import frc.robot.commands.drive.RobotGotoAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drive.RobotGotoFieldPos;
import frc.robot.commands.shooter.TrapAlign;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

class autoSpeaker extends SequentialCommandGroup  {

    public autoSpeaker(ShooterIndexSubsystem shooterIndexSubsystem, DriveSubsystem driveSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, AutoAim autoAim){
        addCommands(
            new ParallelCommandGroup(
                new RobotGotoFieldPos(driveSubsystem, 40, 10, 90), // x y and z of the postion the robot should be in to score the amp, depends on alliance??
                new InstantCommand(() -> autoAim.shooterPivotToSpeakerField())
            ));
    }
}
