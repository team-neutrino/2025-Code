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

import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private SparkFlex m_armMotor = new SparkFlex(0, null); // set up motor later
  private SparkFlexConfig m_armMotorConfig = new SparkFlexConfig();
  private AbsoluteEncoder m_armEncoder;

  public Arm() {
  }

  private double adjustAngleOut(double angle) {
    if (angle > 180) {
      angle -= 360;
    }
    return angle;
  }

  public double getArmPosition() {
    return adjustAngleOut(m_armEncoder.getPosition());
  }

  // finds how far you need to move to be at the desired position
  public double findArmMovementAmount(double desiredPosition) {
    return desiredPosition - getArmPosition();
  }

  // sets up motor controllers
  public void initializeMotorControllers() {
  }

  // finds out if the arm is in limit
  public boolean isArmInLimit() {
    if (!isArmHitingElevator() && !isArmHitingBase()) {
      return true;
    } else {
      return false;
    }
  }

  // add elevator and wrist parts later
  // finds if the arm will be in the area the elevator is in
  public boolean isArmHitingElevator() {
    if (ArmConstants.HITING_LEFT_ELEVATOR_ARM_POSITION < getArmPosition()
        && getArmPosition() < ArmConstants.HITING_RIGHT_ELEVATOR_ARM_POSITION) {
      return true;
    } else {
      return false;
    }
  }

  // add elevator and wrist parts later
  // finds if the arm will be in the area the base is in
  public boolean isArmHitingBase() {
    if (ArmConstants.HITING_LEFT_BASE_ARM_POSITION < getArmPosition()
        && getArmPosition() < ArmConstants.HITING_RIGHT_BASE_ARM_POSITION) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command ArmDefaultCommand() {
    return run(() -> {
    });
  }

  // move the arm a desired amount
  public Command ArmMoveCommand(double movementAmount) {
    return run(() -> {
    });
  }

}
