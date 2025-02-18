// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.States;
import frc.robot.util.Subsystem;

public class LED extends SubsystemBase {

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  StringTopic color_topic = inst.getStringTopic("/LED/color");
  StringTopic state_topic = inst.getStringTopic("/LED/state");
  Coral coral = Subsystem.coral;

  final StringPublisher color_pub;
  final StringPublisher state_pub;

  public LED() {
    color_pub = color_topic.publish();
    state_pub = state_topic.publish();
  }

  public Command LEDefaultCommand() {
    return run(() -> setColor());
  }

  public void setToGamePieceColor() {
    if (coral.hasCoral()) {
      state_pub.set("blinktwice");
      color_pub.set("white");
    }
  }

  public void setColor() {
    if (DriverStation.isAutonomousEnabled()) {
      color_pub.set("cyan");
    } else if (DriverStation.isTeleopEnabled()) {
      if (coral.debouncedHasCoral()) {
        setToGamePieceColor();
        return;
      } else {
    }
  }

  public void setCommandState(States p_state) {
    m_state = p_state;
  }

  public States getCommandState() {
    return m_state;
  }

  @Override
  public void periodic() {
    if (Subsystem.swerve.m_isDrivingToPoint) {
      color_pub.set("red");
      state_pub.set("solid");
    } else if (Subsystem.swerve.m_isAtPoint) {
      color_pub.set("green");
      state_pub.set("solid");
    } else {
      color_pub.set("orange");
      state_pub.set("solid");
    }
  }
}
