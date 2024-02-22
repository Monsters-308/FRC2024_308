package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterPivotConstants;

import com.revrobotics.CANSparkBase;

public class ShooterPivotSubsystem_old extends SubsystemBase {

  private final CANSparkMax shooterPivotMotor = new CANSparkMax(ShooterPivotConstants.kShooterPivotMotorCanID, MotorType.kBrushed);
  private final AbsoluteEncoder shooterPivotMotorEncoder;
  private final SparkPIDController shooterPivotMotorPIDController;

  private final ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

  /** Creates a new ShooterPivotSubsystem. */
  public ShooterPivotSubsystem_old() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    shooterPivotMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the SPARKS MAX.
    shooterPivotMotorEncoder = shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterPivotMotorPIDController = shooterPivotMotor.getPIDController();
    shooterPivotMotorPIDController.setFeedbackDevice(shooterPivotMotorEncoder);

    // Apply position conversion factor for the encoder. The
    // native units for position and velocity are rotations and RPM, respectively.
    // TODO: SwerveModule.java sets both position factor and velocity, but I don't know why both would be required if we're only doing position control
    shooterPivotMotorEncoder.setPositionConversionFactor(ShooterPivotConstants.kShooterEncoderPositionFactor);
    //shooterPivotMotorEncoder.setVelocityConversionFactor(ShooterPivotConstants.kShooterEncoderVelocityFactor);

    // Set the PID gains for the turning motor.
    // shooterPivotMotorPIDController.setP(ShooterPivotConstants.kTurningP);
    // shooterPivotMotorPIDController.setI(ShooterPivotConstants.kTurningI);
    // shooterPivotMotorPIDController.setD(ShooterPivotConstants.kTurningD);
    // shooterPivotMotorPIDController.setFF(ShooterPivotConstants.kTurningFF);
    // shooterPivotMotorPIDController.setOutputRange(ShooterPivotConstants.kTurningMinOutput,
    //     ShooterPivotConstants.kTurningMaxOutput);

    shooterPivotMotor.setIdleMode(ShooterPivotConstants.kTurningMotorIdleMode);
    shooterPivotMotor.setSmartCurrentLimit(ShooterPivotConstants.kTurningMotorCurrentLimit);

    // Inverting turning motor
    shooterPivotMotor.setInverted(ShooterPivotConstants.kTurningMotorInverted);

    // Inverting turning encoder
    shooterPivotMotorEncoder.setInverted(ShooterPivotConstants.kTurningMotorEncoderInverted);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    shooterPivotMotor.burnFlash();

    pivotTab.addDouble("Pivot Angle", () -> getPosition());
  }

  /**
   * Returns the angle the shooter is at.
   * The angle is 0 when the shooter is parallel to the ground.
   *
   * @return The current position of the module (in radians).
   */
  public double getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return shooterPivotMotorEncoder.getPosition();
  }

  /**
   * Set the shooter to a specific angle (in radians)
   * @param angle The angle to set the shooter to (in radians)
   */
  public void setPosition(double angle) {

    // restrict angle from breaking robot
    angle = MathUtil.clamp(angle, ShooterPivotConstants.kPivotMinAngle, ShooterPivotConstants.kPivotMaxAngle);

    shooterPivotMotorPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
