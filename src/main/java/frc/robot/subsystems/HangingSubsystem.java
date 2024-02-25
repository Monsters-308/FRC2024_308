package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangingConstants;

public class HangingSubsystem extends SubsystemBase {

  private final HangingArm m_rightArm = new HangingArm(HangingConstants.kRightArmCanID,
                                                       HangingConstants.kRightArmUpperLimit, 
                                                       HangingConstants.kRightArmLowerLimit,
                                                       HangingConstants.kRightArmInverted);
  private final HangingArm m_leftArm = new HangingArm(HangingConstants.kLeftArmCanID,
                                                      HangingConstants.kLeftArmUpperLimit, 
                                                      HangingConstants.kLeftArmLowerLimit,
                                                      HangingConstants.kLeftArmInverted);
  
  
  private final ShuffleboardTab hangingTab = Shuffleboard.getTab("Hanging");
    
  /** Creates a new HangingSubsystem. */
  public HangingSubsystem() {
    // Shuffleboard values
    hangingTab.addBoolean("LeftUpperLimit", () -> m_leftArm.isFullyExtended());
    hangingTab.addBoolean("LeftLowerLimit", () -> m_leftArm.isFullyRetracted());
    
    hangingTab.addBoolean("RightUpperLimit", () -> m_rightArm.isFullyExtended());
    hangingTab.addBoolean("RightLowerLimit", () -> m_rightArm.isFullyRetracted());
    
  }

  /** Set speed of left arm */
  public void setLeftSpeed(double speed){
    m_leftArm.setSpeed(speed);
  }

  /** Set speed of right arm */
  public void setRightSpeed(double speed){
    m_rightArm.setSpeed(speed);
  }

  /** Set speed of both arms */
  public void setBothSpeed(double speed){
    m_leftArm.setSpeed(speed);
    m_rightArm.setSpeed(speed);
  }

  public boolean leftFullyExtended(){
    return m_leftArm.isFullyExtended();
  }

  public boolean leftFullyRetracted(){
    return m_leftArm.isFullyRetracted();
  }

  public boolean rightFullyExtended(){
    return m_rightArm.isFullyRetracted();
  }

  public boolean rightFullyRetracted(){
    return m_rightArm.isFullyRetracted();
  }

  @Override
  public void periodic() {
    // Safety: stop the motors if they are moving while at extension/retraction limit.
    if (((m_leftArm.isFullyExtended()) && (m_leftArm.getSpeed() > 0))
    ||  ((m_leftArm.isFullyRetracted()) && (m_leftArm.getSpeed() < 0))){
      m_leftArm.setSpeed(0);
    }

    if (((m_rightArm.isFullyExtended()) && (m_rightArm.getSpeed() > 0))
    ||  ((m_rightArm.isFullyRetracted()) && (m_rightArm.getSpeed() < 0))){
      m_rightArm.setSpeed(0);
    }
  }

}
