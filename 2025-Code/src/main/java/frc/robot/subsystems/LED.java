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
    if (coral.isAlgae()) {
      state_pub.set("blinktwice");
      color_pub.set("turquoise");
      // color_pub.set(action:"blink", color:"turquoise");
    } else if (coral.isCoral()) {
      state_pub.set("blinktwice");
      color_pub.set("white");
    }
  }

  public void setColor() {
    if (DriverStation.isAutonomousEnabled()) {
      color_pub.set("cyan");
    } else if (DriverStation.isTeleopEnabled()) {
      if (coral.hasGamePiece()) {
        setToGamePieceColor();
        return;
      } else {
        color_pub.set("orange");
        state_pub.set("solid");
      }
    }
  }

  @Override
  public void periodic() {
  }
}
