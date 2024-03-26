package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Import this so you can make this class a command
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;
import frc.utils.FieldUtils;
import frc.utils.OdometryUtils;

public class AutoAimDynamic extends Command {

    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    final LEDSubsystem m_LedSubsystem;
     
    private final PIDController angleController = new PIDController(HeadingConstants.kHeadingP, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);
    
    //If you want to contoll whether or not the command has ended, you should store it in some sort of variable:
    private boolean m_complete = false;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;

    /**
     * This command rotates the robot in space using the pose estimator compared to the field element pose.
     * The driver still has full control over the X and Y of the robot.
     * @param visionSubsystem
     * @param driveSubsystem
     * @param xSpeed
     * @param ySpeed
     */
    public AutoAimDynamic(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, LEDSubsystem ledSubsystem){
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;

        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(HeadingConstants.kHeadingTolerance);

        m_LedSubsystem = ledSubsystem;

        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.
        addRequirements(driveSubsystem);
    }

    /*This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set m_complete to false in case a command object is 
     * called a second time.
     */
    //When not overridden, this function is blank.
    @Override
    public void initialize(){
        m_visionSubsystem.setPipeline(VisionConstants.kAprilTagPipeline);
        angleController.reset();
        m_complete = false;
        m_LedSubsystem.setLEDFunction(m_LedSubsystem::purple);
    }

    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
    @Override
    public void execute(){
        
        Translation2d pos1 = m_driveSubsystem.getPose().getTranslation(); // Position of robot on field
        Translation2d pos2 = FieldUtils.flipRed(FieldConstants.kSpeakerPosition); //speaker position 
        Rotation2d angleToTarget = OdometryUtils.anglePoseToPose(pos1, pos2); // Angle to make robot face speacker
        double distanceToTarget = OdometryUtils.getDistancePosToPos(pos1, pos2); //distance in inches from limelight to speaker

        SmartDashboard.putNumber("Distance to goal", distanceToTarget);
        SmartDashboard.putNumber("Angle to goal", angleToTarget.getDegrees());

        // Set pid controller to angle to make robot face speaker
        angleController.setSetpoint(angleToTarget.getDegrees());
        
        double robotHeading = m_driveSubsystem.getHeading(); //navx

        double rotation = angleController.calculate(robotHeading); //speed needed to rotate robot to set point

        rotation = MathUtil.clamp(rotation, -HeadingConstants.kHeadingMaxOutput, HeadingConstants.kHeadingMaxOutput); // clamp value (speed limiter)
        
        m_driveSubsystem.drive(
            -MathUtil.applyDeadband(m_xSpeed.getAsDouble(), OIConstants.kJoystickDeadband),
            -MathUtil.applyDeadband(m_ySpeed.getAsDouble(), OIConstants.kJoystickDeadband),
            rotation,
            true, true
        );
            
    }


    /*This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()" function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    @Override
    public void end(boolean interrupted){
        m_LedSubsystem.setLEDFunction(m_LedSubsystem::rainbow);
    }

    @Override
    public boolean isFinished(){
        return m_complete;
    }
}