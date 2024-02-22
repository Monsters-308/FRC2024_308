package frc.robot.commands.drive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drive.RobotGotoFieldPos;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class autoTrapShoot extends ParallelCommandGroup  {

    public autoTrapShoot(DriveSubsystem driveSubsystem){
        addCommands(
            new RobotGotoFieldPos(driveSubsystem, 40, 10, 90) // x y and z of the postion the robot should be in to score the amp, depends on alliance??
        );
    }
}
