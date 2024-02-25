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
import frc.robot.subsystems.ShooterPivotSubsystem;

//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;
import frc.utils.OdometryUtils;
//import frc.utils.ShooterUtils;

public class AutoAimDynamic extends Command {

    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    final ShooterPivotSubsystem m_shooterPivotSubsystem;
     
    private final PIDController angleController = new PIDController(HeadingConstants.kHeadingP, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);
    
    //If you want to contoll whether or not the command has ended, you should store it in some sort of variable:
    private boolean m_complete = false;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;

    /**
     * This command rotates the robot in space and sets the shooter pivot to score using the pose estimator compared to the field element pose
     * You still have full control over the X and Y of the robot
     * @param visionSubsystem
     * @param driveSubsystem
     * @param xSpeed
     * @param ySpeed
     */
    public AutoAimDynamic(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed){
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_shooterPivotSubsystem = shooterPivotSubsystem;


        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.
        addRequirements(driveSubsystem, shooterPivotSubsystem);
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
    }

    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
    @Override
    public void execute(){
        double angle = m_driveSubsystem.getHeading(); //navx
        
        Translation2d pos1 = m_driveSubsystem.getPose().getTranslation(); // Position of robot on field
        Translation2d pos2 = FieldConstants.kSpeakerPosition; //speaker position 
        Rotation2d angleToTarget = OdometryUtils.anglePoseToPose(pos1, pos2); // Angle to make robot face speacker
        double distanceToTarget = OdometryUtils.getDistacnePosToPos(pos1, pos2); //distance in inches from limelight to speaker

       // Shuffleboard.getTab("Vision").add("Angle to Goal", angleToTarget.getDegrees());
       // Shuffleboard.getTab("Vision").add("Distance to Goal", distanceToTarget);

        SmartDashboard.putNumber("Distance to goal", distanceToTarget);
        SmartDashboard.putNumber("Angle to goal", angleToTarget.getDegrees());
        //SmartDashboard.putNumber("limelightX", m_visionSubsystem.getX());




        angleController.setSetpoint(angleToTarget.getDegrees());

        /* 
            get shooter to be at the same angle
            1. Get absolute encoder value from the shooter 
            2. Move the shooter to meet that pot value, do this by using PID to go to the angleToTarget
            3. Shoot the note at the fastest possible speed, lets see how it works
        */
        // double encoderValue = 0;
        // double offset = 1; 
        // double speedAngleChange = 0;
        // double maxDistanceShot = 20;

        

        double rotation = angleController.calculate(angle); //speed needed to rotate robot to set point

        rotation = MathUtil.clamp(rotation, -HeadingConstants.kHeadingMaxOutput, HeadingConstants.kHeadingMaxOutput); // clamp value (speed limiter)

        
        m_driveSubsystem.drive(
            -MathUtil.applyDeadband(m_xSpeed.getAsDouble(), OIConstants.kJoystickDeadband),
            -MathUtil.applyDeadband(m_ySpeed.getAsDouble(), OIConstants.kJoystickDeadband),
            rotation,
            true, true
        );
            
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
    
    /**
     * Gets the proper pivot to angle the shooter at the speaker according the the m_driveSubsystem.getPose() and FieldConstants.kSpeakerPosition, FieldConstants.kSpeakerHeight
     * @return The angle to pivot towards the speaker as a double
     */
    // public void shooterPivotToSpeakerField(){
    //     //change second parameter to shooter util robot pos
    //     double anglePivot = ShooterUtils.shooterAngleToFacePoint(m_driveSubsystem.getPose().getTranslation(), FieldConstants.kSpeakerPosition, FieldConstants.kSpeakerHeight);

    //     m_shooterPivotSubsystem.setPosition(anglePivot);
    // }

    /*This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()" function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    @Override
    public void end(boolean interrupted){
    
    }

    @Override
    public boolean isFinished(){
        return m_complete;
    }
}