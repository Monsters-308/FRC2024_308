package frc.robot.commands.vision;

import frc.robot.subsystems.DriveSubsystem;

//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.VisionConstants;

//Import this so you can make this class a command
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignAutoAim extends Command {

    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    final PIDController distanceController = new PIDController(.2, 0, 0.05);

    boolean isAlignDistacne = false;
    boolean isAlignRotation = false;
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    //If you want to contoll whether or not the command has ended, you should store it in some sort of variable:
    private boolean m_complete = false;

    //Class Constructor
    public AutoAlignAutoAim(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem){
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        
        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.
        addRequirements(m_visionSubsystem);
    }



    /*Like Robot.java, there are a series of functions that you can override to give the command functionality. */
    

    /*This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set m_complete to false in case a command object is 
     * called a second time.
     */
    //When not overridden, this function is blank.
    @Override
    public void initialize(){
        //m_chassisSubsystem.setBrakeMode();
        m_visionSubsystem.setPipeline(VisionConstants.kReflectiveTapePipeline);
        m_complete = false;
    }

    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
    @Override
    public void execute(){
        double y = m_visionSubsystem.getY();
        double x = m_visionSubsystem.getX();
        double targets = m_visionSubsystem.getTV();
        double forwardSpeed = 0;
        double rotate = 0;
        double angle = m_gyro.getAngle(); 
        double crabCrawl = 0;
        //SmartDashboard.putNumber("motor speed align targets", x);

        if ((targets == 0) || (y > 0)){
            rotate = 0;
            forwardSpeed = 0;
            //m_ledSubsystem.changeLEDState(LEDState.RED);
        }

        else{
            //this returns how fast the robot shoud move in order to get to middle
            if (Math.abs(angle) > .1){
                double normalizedAngle = (angle + 180) % 360 - 180;
                crabCrawl = (normalizedAngle / 180);
            }

            //Rotate so target is in center
            if (x+1 > VisionConstants.kRotationTolerance){
                rotate = VisionConstants.kRotationSpeed;//.6
            }
            else if (x+1 < -VisionConstants.kRotationTolerance){
                rotate = -VisionConstants.kRotationSpeed;
            }

            //Move forwards/backwards
            double distanceFromTarget = m_visionSubsystem.getReflectiveTapeDistance();

            //P controller for distance (fred)
            //forwardSpeed = (currentPosition - desiredPosition) * Pconstant
            if ((distanceFromTarget < VisionConstants.kTopPoleDesiredDistance - VisionConstants.kDistanceTolerance) 
            || (distanceFromTarget > VisionConstants.kTopPoleDesiredDistance + VisionConstants.kDistanceTolerance)){
                forwardSpeed = (m_visionSubsystem.getReflectiveTapeDistance() - VisionConstants.kTopPoleDesiredDistance) * VisionConstants.kForwardSpeedPConstant;
            }
            else{
                forwardSpeed = 0;
            }

            //Change LED state
            if((distanceFromTarget > 39) && (distanceFromTarget < 41)){
                isAlignDistacne = true;
            }
            else{
                isAlignDistacne = false;
            }
            if ((x+1 < VisionConstants.kRotationTolerance)&&(x+1 > -VisionConstants.kRotationTolerance)){
                isAlignRotation = true;
            }
            else{
                isAlignRotation = false;
            }
        }
        //PID
        forwardSpeed = distanceController.calculate(forwardSpeed);
        crabCrawl = distanceController.calculate(crabCrawl);
        SmartDashboard.putNumber("motor speed align forwardspeed", forwardSpeed);
        SmartDashboard.putNumber("motor speed align rotation", rotate);

        m_driveSubsystem.drive(forwardSpeed*.2, crabCrawl*.2, rotate*.2, false, true);
  
    
        
    }

    /*This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()" function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    @Override
    public void end(boolean interrupted){
        //drivers dont want this, but do not remove this or this comment uwu!
        //m_chassisSubsystem.drive(0, 0);

    }

    @Override
    public boolean isFinished(){
        return m_complete;
    }
}