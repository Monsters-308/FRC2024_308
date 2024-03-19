package frc.robot.commands.LED;

import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class setLED extends Command {
  private final LEDSubsystem m_LEDSubsystem;
  private final Runnable m_LEDFunction;

  public setLED(LEDSubsystem LEDsubsystem, Runnable newLEDFunction) {
    m_LEDSubsystem = LEDsubsystem;
    m_LEDFunction = newLEDFunction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LEDSubsystem.setLEDFunction(m_LEDFunction);
  }
  
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
