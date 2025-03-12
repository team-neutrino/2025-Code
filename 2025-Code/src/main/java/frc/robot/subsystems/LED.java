// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import static frc.robot.util.Subsystem.*;

public class LED extends SubsystemBase {

  private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
  private StringTopic m_colortopic = m_nt.getStringTopic("/LED/color");
  private StringTopic m_statetopic = m_nt.getStringTopic("/LED/state");
  private Coral m_coral = coral;

  private final StringPublisher m_colorpub;
  private final StringPublisher m_statepub;

  public LED() {
    m_colorpub = m_colortopic.publish();
    m_statepub = m_statetopic.publish();
  }

  private void setStateAndColor(String state, String color){
    m_statepub.set(state);
    m_colorpub.set(color);
  }
  
  @Override
  public void periodic() {
    if(swerve.isDrivingToPoint()) {
      if(!elevator.atTargetHeight() && !arm.atTargetAngle()) {
        setStateAndColor("solid", "cyan");
      }
      else if (!elevator.atTargetHeight()) {
        setStateAndColor("solid", "purple");
      } 
      else if (!arm.atTargetAngle()) {
        setStateAndColor("solid", "turquoise");
      }
      else {
        setStateAndColor("solid", "red");
      }
      return;
    }
    else {
      if(!elevator.atTargetHeight() && !arm.atTargetAngle()) {
        setStateAndColor("solid", "pink");
        return;
      }
      else if (!elevator.atTargetHeight()) {
        setStateAndColor("solid", "blue");
        return;
      }
      else if (!arm.atTargetAngle()) {
        setStateAndColor("solid", "yellow");
      }
    }

    if (swerve.isAtPoint() && elevator.readyToScore() && arm.readyToScore()) {
      setStateAndColor("solid", "green");
      return;
    }

  if (m_coral.debouncedHasCoral()) {
    setStateAndColor("blinktwice", "white");
      return;
    }

    // default to orange
    setStateAndColor("solid", "orange");
  }
}