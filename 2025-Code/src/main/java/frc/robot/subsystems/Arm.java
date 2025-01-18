// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants.ArmConstants;
import frc.robot.command_factories.ArmFactory;

public class Arm extends SubsystemBase {

  private SparkFlex m_armMotor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
  private SparkFlexConfig m_armMotorConfig = new SparkFlexConfig();
  private AbsoluteEncoder m_armEncoder;
  private SparkClosedLoopController m_armPidController;

  private double m_targetAngle = 0;

  public Arm() {
    initializeMotorControllers();
  }

  public double getArmPosition() {
    return m_armEncoder.getPosition();
  }

  // sets up motor controllers
  public void initializeMotorControllers() {
    m_armEncoder = m_armMotor.getAbsoluteEncoder();
    m_armPidController = m_armMotor.getClosedLoopController();
    m_armMotorConfig.idleMode(IdleMode.kBrake);

    m_armMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    m_armMotorConfig.signals.absoluteEncoderPositionPeriodMs(5);

    m_armMotorConfig.smartCurrentLimit(ArmConstants.CURRENT_LIMIT);

    m_armMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ArmConstants.kp, ArmConstants.ki, ArmConstants.kd, ClosedLoopSlot.kSlot0);
    m_armPidController = m_armMotor.getClosedLoopController();

    m_armMotor.configure(m_armMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // finds out if the arm is in limit
  public boolean isArmInLimit() {
    return true;
  }

  public void updateArmAngle() {
    System.out.println(m_targetAngle);
    m_armPidController.setReference(m_targetAngle,
        SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {
    updateArmAngle();
  }

  public Command armDefaultCommand() {
    return run(() -> m_targetAngle = DEFAULT_ARM_POSITION);
  }

  // move the arm a desired amount
  public Command armRotateCommand(double targetAngle) {
    return run(() -> m_targetAngle = targetAngle);
  }

}
