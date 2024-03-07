package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterIndexConstants;
import frc.robot.commands.shooterIndex.RunShooterIndexer;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterIndexSubsystem extends SubsystemBase {

    private final CANSparkMax m_conveyorMotor = new CANSparkMax(ShooterIndexConstants.kMotorCanID, MotorType.kBrushed);
    
    private final DigitalInput m_noteSensor = new DigitalInput(ShooterIndexConstants.kDigitalSensorPin);

    private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

    /** 
     * Controls the conveyor belt that brings the note to the shooter flywheels, along with the 
     * light sensor for detecting if a note is in the shooter.  
    */
    public ShooterIndexSubsystem() {
        m_conveyorMotor.restoreFactoryDefaults();

        m_conveyorMotor.setInverted(ShooterIndexConstants.kInvertMotor);

        m_conveyorMotor.setSmartCurrentLimit(ShooterIndexConstants.kMotorCurrentLimit);

        m_conveyorMotor.setIdleMode(IdleMode.kBrake);

        m_conveyorMotor.burnFlash();

        shooterTab.addBoolean("Game Piece", () -> gamePieceDetected());
        
        shooterTab.add("Run Indexer", new RunShooterIndexer(this, 0.5));
    }

    /**
    * Sets the motor to a certain speed.
    * @param speed A speed from -1 to 1.
    */
    public void setSpeed(double speed){
        m_conveyorMotor.set(speed);
    }

    /**
    * Returns whether or not there's a game piece in the intake.
    * pancake pancake pancake pancake pancake pancake pancake pancake
    * @return true if the sensor detects a game piece.
    */
    public boolean gamePieceDetected(){
        if (ShooterIndexConstants.kSensorInverted){
            return !m_noteSensor.get();
        }
        return m_noteSensor.get();
    }  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
