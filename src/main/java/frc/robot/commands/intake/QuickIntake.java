package frc.robot.commands.intake;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterIndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.commands.shooterIndex.IDK;
import frc.robot.Constants.ShooterIndexConstants;
    
public class QuickIntake extends ParallelRaceGroup {


  public QuickIntake(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem, ShooterIndexSubsystem shooterIndexSubsystem) {
    deadlineWith(new IDK(shooterIndexSubsystem));

  }
}
