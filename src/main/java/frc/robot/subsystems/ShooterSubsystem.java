package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax m_topShooterMotor = new CANSparkMax(ShooterConstants.kTopShooterMotorCanID, MotorType.kBrushless);
  private final CANSparkMax m_bottomShooterMotor = new CANSparkMax(ShooterConstants.kBottomShooterMotorCanID, MotorType.kBrushless);

  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

  private final RelativeEncoder shooterMotorEncoderTop;
  private final RelativeEncoder shooterMotorEncoderBottom;

  private final SparkPIDController shooterMotorTopPIDController;
  private final SparkPIDController shooterMotorBottomPIDController;

  /** 
   * Controls the two shooter flywheels and uses the relative encoders built 
   * into the neos for velocity control. 
   */
  public ShooterSubsystem() {
    m_topShooterMotor.restoreFactoryDefaults(); 
    m_bottomShooterMotor.restoreFactoryDefaults(); 

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    shooterMotorEncoderTop = m_topShooterMotor.getEncoder();
    shooterMotorEncoderBottom = m_bottomShooterMotor.getEncoder();

    shooterMotorTopPIDController = m_topShooterMotor.getPIDController();
    shooterMotorBottomPIDController = m_bottomShooterMotor.getPIDController();

    shooterMotorTopPIDController.setFeedbackDevice(shooterMotorEncoderTop);
    shooterMotorBottomPIDController.setFeedbackDevice(shooterMotorEncoderBottom);

    // Converts the default unit of RPM to meters per second. 
    // Making this a linear velocity allows us to estimate the velocity of the note as it exits the shooter (as opposed to making this a rotational velocity)
    shooterMotorEncoderTop.setVelocityConversionFactor(ShooterConstants.kDrivingEncoderVelocityFactor);
    shooterMotorEncoderBottom.setVelocityConversionFactor(ShooterConstants.kDrivingEncoderVelocityFactor);

    // Set the PID gains for the turning motor.
    shooterMotorTopPIDController.setP(ShooterConstants.kShooterP);
    shooterMotorTopPIDController.setI(ShooterConstants.kShooterI);
    shooterMotorTopPIDController.setD(ShooterConstants.kShooterD);
    shooterMotorTopPIDController.setFF(ShooterConstants.kShooterFF);
    shooterMotorTopPIDController.setOutputRange(ShooterConstants.kShooterMinOutput,
        ShooterConstants.kShooterMaxOutput);
    //bottom
    shooterMotorBottomPIDController.setP(ShooterConstants.kShooterP);
    shooterMotorBottomPIDController.setI(ShooterConstants.kShooterI);
    shooterMotorBottomPIDController.setD(ShooterConstants.kShooterD);
    shooterMotorBottomPIDController.setFF(ShooterConstants.kShooterFF);
    shooterMotorBottomPIDController.setOutputRange(ShooterConstants.kShooterMinOutput,
        ShooterConstants.kShooterMaxOutput);

    // Limit the current so the motor doesn't oof itself
    m_topShooterMotor.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit); 
    m_bottomShooterMotor.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit);
    
    //setting motors to spin freely (like ur mummy)
    m_topShooterMotor.setIdleMode(IdleMode.kCoast); 
    m_bottomShooterMotor.setIdleMode(IdleMode.kCoast);

    // Invert motors
    m_topShooterMotor.setInverted(ShooterConstants.kInvertTopMotor);
    m_bottomShooterMotor.setInverted(ShooterConstants.kInvertBottomMotor);

    // Save the motor controller's configuration
    m_topShooterMotor.burnFlash(); 
    m_bottomShooterMotor.burnFlash();

    setPercent(ShooterConstants.kIdleRevSpeed);

    // Add Values to shuffleboard
    shooterTab.addDouble("Top Roller Speed", () -> getTopSpeed());
    shooterTab.addDouble("Bottom Roller Speed", () -> getBottomSpeed());

    shooterTab.addDouble("Top Roller percent", () -> m_topShooterMotor.getAppliedOutput());
    shooterTab.addDouble("Bottom Roller percent", () -> m_bottomShooterMotor.getAppliedOutput());
  }

  /**
   * Set the top shooter roller to a specific speed using PID.
   * @param speed The speed in meters per second.
   */
  public void setTopShooterSpeed(double speed){
    shooterMotorTopPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
  } 

  /**
   * Set the bottom shooter roller to a specific speed using PID.
   * @param speed The speed in meters per second.
   */
  public void setBottomShooterSpeed(double speed){
    shooterMotorBottomPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  /**
   * Set both shooter rollers to a specific speed using PID.
   * @param speed The speed in meters per second.
   */
  public void setBothSpeeds(double speed){
    shooterMotorTopPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
    shooterMotorBottomPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  /** 
   * Stops the shooter rollers.
   * NOTE: unlike setBothSpeeds(0), this lets the wheels naturally wind down.
   */
  public void stopRollers(){
    m_topShooterMotor.set(ShooterConstants.kIdleRevSpeed);
    m_bottomShooterMotor.set(ShooterConstants.kIdleRevSpeed);
  }

  //NOTE: this function is for testing purposes
  public void setPercent(double speed){
    m_topShooterMotor.set(speed);
    m_bottomShooterMotor.set(speed);
  }

  /** Returns the speed of the top shooter roller in meters per second. */
  public double getTopSpeed(){
    return shooterMotorEncoderTop.getVelocity();
  }

  /** Returns the speed of the bottom shooter roller in meters per second. */
  public double getBottomSpeed(){
    return shooterMotorEncoderBottom.getVelocity();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}