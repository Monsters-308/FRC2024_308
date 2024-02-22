package frc.robot.commands.commandGroups.drive;
import frc.robot.commands.drive.RobotGotoFieldPos;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterIndexSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.utils.FieldUtils;

public class autoSpeaker extends SequentialCommandGroup  {

    /**  */
    public autoSpeaker(DriveSubsystem driveSubsystem){
        addCommands(
            new ParallelCommandGroup(
                new RobotGotoFieldPos(driveSubsystem, 40, FieldUtils.flipRedY(69), FieldUtils.flipRedAngle(-90)) // x y and z of the postion the robot should be in to score the amp, depends on alliance??
            ));
    }
}
