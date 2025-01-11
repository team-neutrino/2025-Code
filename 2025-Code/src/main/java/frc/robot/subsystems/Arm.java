// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;

public class Arm extends SubsystemBase {

  private SparkFlex m_armMotor = new SparkFlex(0, null);
  private SparkFlexConfig m_armMotorConfig = new SparkFlexConfig();
  private AbsoluteEncoder m_encoder;

  public Arm() {
  }

  private double adjustAngleOut(double angle) {
    if (angle > 180) {
      angle -= 360;
    }
    return angle;
  }

  public double getArmPosition() {
    return adjustAngleOut(m_encoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command ArmDefaultCommand() {
    return Commands.runOnce(() -> {
    });
  }

  public Command ArmMoveCommand(double movementAmount) {
    return Commands.runOnce(() -> {
    });
  }

}
