// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.util.Subsystem;

public class LED extends SubsystemBase {

  private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
  private StringTopic m_colortopic = m_nt.getStringTopic("/LED/color");
  private StringTopic m_statetopic = m_nt.getStringTopic("/LED/state");
  private Coral m_coral = Subsystem.coral;

  private final StringPublisher m_colorpub;
  private final StringPublisher m_statepub;

  public LED() {
    m_colorpub = m_colortopic.publish();
    m_statepub = m_statetopic.publish();
  }
  
  @Override
  public void periodic() {
    if(Subsystem.swerve.isDrivingToPoint() && !Subsystem.elevator.atTargetHeight() && !Subsystem.arm.atTargetAngle()){
      m_statepub.set("solid");
      m_colorpub.set("cyan");
      return;
    } else if (Subsystem.swerve.isDrivingToPoint() && !Subsystem.elevator.readyToScore()) {
      m_statepub.set("solid");
      m_colorpub.set("purple");
      return;
    }else if (Subsystem.swerve.isDrivingToPoint()) {
      m_statepub.set("solid");
      m_colorpub.set("red");
      return;
    }

    if (Subsystem.swerve.isAtPoint() && Subsystem.elevator.readyToScore() && Subsystem.arm.readyToScore()) {
      m_colorpub.set("green");
      m_statepub.set("solid");
      return;
    }

  if (m_coral.debouncedHasCoral()) {
      m_statepub.set("blinktwice");
      m_colorpub.set("white");
      return;
    }

    // default to orange
    m_colorpub.set("orange");
    m_statepub.set("solid");
  }
}