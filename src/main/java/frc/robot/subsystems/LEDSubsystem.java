package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class LEDSubsystem extends SubsystemBase {
 
  private Runnable activeFunction; // Reference to the currently active LED function
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int[] LEDColor = {255, 0, 0}; //RGB
  private int m_rainbowFirstPixelHue = 0;
  private int m_policeLights = 0;
  private double robotPitch;
  private int m_TurboFirstPixelHue = 0;
  private int rainbowSunshine = 0;

  public LEDSubsystem(DoubleSupplier Pitch){
    robotPitch = Pitch.getAsDouble();
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(125);
    
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    
    activeFunction = this::rainbow;    
  }

  @Override
  public void periodic() {    
    activeFunction.run();

    m_rainbowFirstPixelHue = Math.abs((int)robotPitch) + m_TurboFirstPixelHue;
  }
    

  public void teal() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 0 ,180);
    }

    m_led.setData(m_ledBuffer);
  }

  public void solid(int[] RGB) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, RGB[0], RGB[1], RGB[2]);
    }

    m_led.setData(m_ledBuffer);
  }
  
  public void green() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void orange() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 255, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void greenAlign(int ammountGreen) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, ammountGreen, 30);
    }

    m_led.setData(m_ledBuffer);
  }

  public void red() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void yellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 255, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void rainbow() {

    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())+ (rainbowSunshine + (i * 180 / m_ledBuffer.getLength()))) % 360;
      // Set the value
      m_ledBuffer.setRGB(i, hue*3, hue*10, rainbowSunshine*5);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }


  public void police() {
    // Calculate the hue based on the overall policeLights value
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        if (m_policeLights > 90) {
            m_ledBuffer.setRGB(i, 0, 0, 255); // Set to blue
        } else {
            m_ledBuffer.setRGB(i, 255, 0, 0); // Set to red
        }
    }

    m_policeLights += 3;
    m_policeLights %= 180;
    m_led.setData(m_ledBuffer);
  }

  public void setLEDFunction(Runnable function) {
    // Set the LED function to be called periodically
    activeFunction = function;
  }
}
