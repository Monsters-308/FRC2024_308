package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.commands.intakePivot.SetIntakeAngle;
import frc.utils.SwerveUtils;

public class IntakePivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_masterMotor = new CANSparkMax(IntakePivotConstants.kMotorCanID, MotorType.kBrushed);
  private final CANSparkMax m_followerMotor = new CANSparkMax(IntakePivotConstants.KOtherMotorCanID, MotorType.kBrushed);

  private final DutyCycleEncoder m_intakePivotMotorEncoder = new DutyCycleEncoder(IntakePivotConstants.kEncoderPort);

  private final ShuffleboardTab m_Tab = Shuffleboard.getTab("Intake");

  /** 
   * Controls the motors that rotate the intake down and up, along with the absolute
   * encoder to detect the rotation of the intake.
   */
  public IntakePivotSubsystem() {
    m_intakePivotMotorEncoder.setDistancePerRotation(IntakePivotConstants.kShooterEncoderPositionFactor);

    m_masterMotor.restoreFactoryDefaults();
    m_followerMotor.restoreFactoryDefaults();

    // Limit the current so the motor doesn't oof itself
    m_masterMotor.setSmartCurrentLimit(IntakePivotConstants.kMotorSmartCurrentLimit);
    m_followerMotor.setSmartCurrentLimit(IntakePivotConstants.kMotorSmartCurrentLimit);

    // Set to brake mode
    m_masterMotor.setIdleMode(IdleMode.kBrake);
    m_followerMotor.setIdleMode(IdleMode.kBrake);

    // Set inverted (positive should be down)
    m_masterMotor.setInverted(IntakePivotConstants.kInvertMotors);
    m_followerMotor.setInverted(!IntakePivotConstants.kInvertMotors);

    // Have motors use the same can ID because it's technically slightly more efficient
    m_followerMotor.follow(m_masterMotor, true);

    // Save the motor controller's configuration
    m_masterMotor.burnFlash();
    m_followerMotor.burnFlash();

    // Shuffleboard values
    m_Tab.addDouble("Pivot Angle", () -> getPosition().getDegrees());

    m_Tab.addBoolean("Encoder Connected", () -> m_intakePivotMotorEncoder.isConnected());

    // Shuffleboard widgets for manually controlling intake pivot
    m_Tab.add("IntakeDown", 
      new SetIntakeAngle(this, IntakePivotConstants.kIntakeDownPosition));
      
    m_Tab.add("IntakeDeck", 
      new SetIntakeAngle(this, IntakePivotConstants.kIntakeDeckPosition));
    
    m_Tab.add("IntakeUp", 
      new SetIntakeAngle(this, IntakePivotConstants.kIntakeInPosition));
  }

  /**
   * Returns the angle the pivot is at.
   * The angle is 0 when the pivot is parallel to the ground.
   *
   * @return The current position of the module (in radians).
   */
  public Rotation2d getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    if(IntakePivotConstants.kTurningMotorEncoderInverted){
      return Rotation2d.fromRadians(-m_intakePivotMotorEncoder.getDistance() + IntakePivotConstants.kAngleOffset);
    }
    return Rotation2d.fromDegrees(SwerveUtils.angleConstrain(m_intakePivotMotorEncoder.getAbsolutePosition()*IntakePivotConstants.kShooterEncoderPositionFactor + IntakePivotConstants.kAngleOffset));
  
  }

  /** This function is for testing purposes */
  public void setSpeed(double speed){
    m_masterMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
  }

}

