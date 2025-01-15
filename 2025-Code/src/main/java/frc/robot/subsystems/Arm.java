// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;

import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private SparkFlex m_armMotor = new SparkFlex(ArmConstants.ARM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private SparkFlexConfig m_armMotorConfig = new SparkFlexConfig();
  private AbsoluteEncoder m_armEncoder;
  private SparkClosedLoopController m_armPidController;

  public Arm() {
    initializeMotorControllers();
  }

  public double getArmPosition() {
    return m_armEncoder.getPosition();
  }

  // finds how far you need to move to be at the desired position
  public double findArmMovementAmount(double desiredPosition) {
    return desiredPosition - getArmPosition();
  }

  // sets up motor controllers
  public void initializeMotorControllers() {
    m_armEncoder = m_armMotor.getAbsoluteEncoder();
    m_armPidController = m_armMotor.getClosedLoopController();
    m_armMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_armMotorConfig.encoder.positionConversionFactor(360);
    m_armMotorConfig.absoluteEncoder.zeroOffset(ArmConstants.ARM_ENCODER_ZERO_OFFSET);

    m_armMotorConfig.smartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);

    m_armMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ArmConstants.Arm_kp, ArmConstants.Arm_ki, ArmConstants.Arm_kd, ClosedLoopSlot.kSlot1)
        .iZone(ArmConstants.ArmIZone)
        .positionWrappingMaxInput(360)
        .positionWrappingMinInput(0)
        .positionWrappingEnabled(true);
    m_armPidController = m_armMotor.getClosedLoopController();
  }

  // finds out if the arm is in limit
  public boolean isArmInLimit() {
    return !isArmHitingElevator() && !isArmHitingBase();
  }

  // add elevator and wrist parts later
  // finds if the arm will be in the area the elevator is in
  public boolean isArmHitingElevator() {
    return ArmConstants.HITING_LEFT_BOTTOM_ELEVATOR_ARM_POSITION < getArmPosition()
        && getArmPosition() < ArmConstants.HITING_RIGHT_BOTTOM_ELEVATOR_ARM_POSITION;
  }

  // add elevator and wrist parts later
  // finds if the arm will be in the area the base is in
  public boolean isArmHitingBase() {
    return ArmConstants.HITING_LEFT_BASE_ARM_POSITION < getArmPosition()
        && getArmPosition() < ArmConstants.HITING_RIGHT_BASE_ARM_POSITION;
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
  public Command ArmMoveCommand(double movementAmount, double targetAngle) {
    return Commands.runOnce(() -> {

    });
  }

}
