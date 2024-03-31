package frc.robot.commands.shooterPivot;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.utils.FieldUtils;
import frc.utils.ShooterUtils;
import edu.wpi.first.wpilibj2.command.Command;
//OwO

public class DynamicPivotToSpeaker extends Command {

  private final ShooterPivotSubsystem m_shooterPivotSubsystem;
  private final DriveSubsystem m_DriveSubsystem;

  /**
   * Makes the shooter pivot auto-aim into the speaker. At the moment, this does not account for shooter 
   * speed and assumes that the note takes a straight path through the air. This also doesn't account for 
   * the speed of the robot.
   * @param pivotSubsystem
   * @param driveSubsystem
   */
  public DynamicPivotToSpeaker(ShooterPivotSubsystem pivotSubsystem, DriveSubsystem driveSubsystem) {
    m_shooterPivotSubsystem = pivotSubsystem;
    m_DriveSubsystem = driveSubsystem;

    // Do not add drive subsystem because we are just using it for data.
    addRequirements(m_shooterPivotSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double anglePivot = ShooterUtils.shooterAngleToFacePoint(
        m_DriveSubsystem.getPose().getTranslation(),
        FieldUtils.flipRed(FieldConstants.kSpeakerPosition)
    );

    m_shooterPivotSubsystem.setPosition(anglePivot);
  }

  // public void shooterPivotToCrosshair(){
  //     double xCrosshairDistance = m_visionSubsystem.getX();
  //     double yCrosshairDistance = m_visionSubsystem.getY();
  //     //Translation2d pos1 = m_driveSubsystem.getPose().getTranslation(); // Position of robot on field
  //     Translation2d pos1 = new Translation2d(0, 0); //assuming the limelight has crosshair offset setup
  //     Translation2d pos2 = new Translation2d(xCrosshairDistance, yCrosshairDistance); //speaker position 

  //     Rotation2d angleToTarget = OdometryUtils.anglePoseToPose(pos1, pos2); // Angle to make robot face speacker


  //     //here we need to calculate the angle required to make a shot


  //     //pivotController.setSetpoint(0);
  //     //double rotation = angleController.calculate(yCrosshairDistance);        
  // }

  @Override
  public void end(boolean interrupted) {
    m_shooterPivotSubsystem.stopMovement();
  }
}