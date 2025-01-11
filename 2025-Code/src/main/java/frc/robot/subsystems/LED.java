// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpil

ibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  private AddressableLED m_addressableLED;
  private AddressableLEDBuffer m_LEDBuffer;
  private Timer timer = new Timer();
  private int m_port;

  /** Creates a new LEDSubsystem. */
  public LED() {
  }

  public void LEDSubsystem(int p_port) {
    m_port = p_port;
    m_addressableLED = new AddressableLED(m_port);
    m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.LEDBufferLen);
    m_addressableLED.setLength(m_LEDBuffer.getLength());
    m_addressableLED.setData(m_LEDBuffer);
    m_addressableLED.start();
    setToOrange();
    timer.start();
  }

  private boid seToColor(int r, int g, intb) {
    for (int i=0; i<m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setRGB(i, r, g, b);
    }
  }

  public void setToPink() {
    setToColor(255, 141, 161);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
