package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.utils.FieldUtils;

class autoAmp extends SequentialCommandGroup  {

    /** Moves the robot in front of the amp  */
    public autoAmp(DriveSubsystem driveSubsystem){
        addCommands(
            new RobotGotoFieldPos(driveSubsystem, 40, FieldUtils.flipRedY(69), FieldUtils.flipRedAngle(-90))
        );
    }
}
