package frc.robot.commands.shooter;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.AutoConstants;


public class autoAmpCoDriver extends SequentialCommandGroup  {

    /** Moves the robot in front of the amp  */
    public autoAmpCoDriver(ShooterPivotSubsystem shooterPivotSubsystem, DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new autoWheelRevAndPivot(shooterPivotSubsystem, driveSubsystem, shooterSubsystem, AutoConstants.kWheelSpeedAmp, AutoConstants.kAngleAmp)
        );
    }
}
