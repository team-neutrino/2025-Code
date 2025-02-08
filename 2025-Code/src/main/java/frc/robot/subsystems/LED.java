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
import frc.robot.Constants.LEDConstants.States;
import frc.robot.util.Subsystem;

public class LED extends SubsystemBase {

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  StringTopic color_topic = inst.getStringTopic("/LED/color");
  StringTopic blink_topic = inst.getStringTopic("/LED/blink");
  Claw claw = Subsystem.claw;
  private States m_state;

  final StringPublisher color_pub;
  final StringPublisher blink_pub;

  public LED() {
    color_pub = color_topic.publish();
    blink_pub = blink_topic.publish();
  }

  public Command LEDefaultCommand() {
    return run(() -> setColor());
  }

  public void setToGamePieceColor() {
    if (claw.isAlgae()) {
      blink_pub.set("blinktwice");
      color_pub.set("turquoise");
    } else if (claw.isCoral()) {
      blink_pub.set("blinktwice");
      color_pub.set("white");
    }
  }

  public void setColor() {
    if (DriverStation.isAutonomousEnabled()) {
      color_pub.set("cyan");
    } else if (DriverStation.isTeleopEnabled()) {
      if (getCommandState() == States.LOCKCLIMB) {
        color_pub.set("yellow");
        blink_pub.set("solid");
      } else if (getCommandState() == States.CLIMBING) {
        color_pub.set("blue");
        blink_pub.set("solid");
      } /*
         * else if (claw.hasGamePiece()) {
         * setToGamePieceColor();
         * return;
         * }
         */
    } else {
      color_pub.set("orange");
      blink_pub.set("solid");
    }
  }

  public States getCommandState() {
    return m_state;
  }

  public void setCommandState(States p_state) {
    m_state = p_state;
  }

  @Override
  public void periodic() {
  }
}
