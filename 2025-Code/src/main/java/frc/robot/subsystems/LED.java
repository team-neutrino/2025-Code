// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.util.Subsystem.claw;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;
import frc.robot.util.Subsystem;

public class LED extends SubsystemBase {
  private int m_counter = 0;
  private int m_counter2 = 0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  StringTopic color_topic = inst.getStringTopic("/LED/color");
  Claw claw = Subsystem.claw;

  final StringPublisher color_pub;

  public LED() {
    color_pub = color_topic.publish();

  }

  private boolean slowDown(int rate) {
    m_counter2++;
    return m_counter2 % rate == 0;
  }

  public Command LEDefaultCommand() {
    return run(() -> setColor());
  }

  // Command or void? What to return(forgot)
  public void setToGamePieceColor() {
    if (claw.isAlgae()) {
      color_pub.set("blinkturquoise");
    } else if (claw.isCoral()) {
      color_pub.set("blinkwhite");
    }
  }

  public void setColor() {
    if (DriverStation.isAutonomousEnabled()) {
      color_pub.set("cyan");
    } else if (DriverStation.isTeleopEnabled()) {
      if (claw.hasGamePiece()) {
        setToGamePieceColor();
        return;
      } else {
        color_pub.set("orange");
      }
    }
  }

  @Override
  public void periodic() {
  }
}
