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
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  private SparkFlex m_armMotor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
  private SparkFlexConfig m_armMotorConfig = new SparkFlexConfig();
  private AbsoluteEncoder m_armEncoder;
  private SparkClosedLoopController m_armPidController;
  private SparkBaseConfigAccessor m_sparkBaseConfigAccessor;
  public ClosedLoopConfigAccessor m_armPidAccessor;

  private double m_targetAngle = 0;

  public Arm() {
    initializeMotorControllers();
    m_sparkBaseConfigAccessor = SOMETHINGDFSJDS
    m_armPidAccessor = m_sparkBaseConfigAccessor.closedLoop;
  }

  public double getArmEncoderPosition() {
    return m_armEncoder.getPosition();
  }

  public double getArmTargetPosition() {
    return m_targetAngle;
  }

  public double getArmVoltage() {
    return m_armMotor.getBusVoltage();
  }

  // sets up motor controllers
  public void initializeMotorControllers() {
    m_armEncoder = m_armMotor.getAbsoluteEncoder();
    m_armPidController = m_armMotor.getClosedLoopController();
    m_armMotorConfig.idleMode(IdleMode.kBrake);

    m_armMotorConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(1);

    m_armMotorConfig.signals.absoluteEncoderPositionPeriodMs(5);

    m_armMotorConfig.smartCurrentLimit(CURRENT_LIMIT);

    m_armMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(kp, ki, kd, ClosedLoopSlot.kSlot0);

    m_armMotorConfig.closedLoop.maxMotion
        .maxVelocity(MAX_VELOCITY)
        .maxAcceleration(MAX_ACCELERATION)
        .allowedClosedLoopError(ALLOWED_ERROR);

    m_armMotor.configure(m_armMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // finds out if the arm is in limit
  public boolean isArmInLimit() {
    return true;
  }

  public void updateArmAngle() {
    m_armPidController.setReference(m_targetAngle,
        SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardCalculation());
  }

  public double feedForwardCalculation() {
    double currentAngle = getArmEncoderPosition();
    double volts = FFCONSTANT * Math.cos(currentAngle);
    return volts;
  }

  public void changePID(double p, double i, double d) {
    m_armMotorConfig.closedLoop.pid(p, i, d);
    m_armMotor.configure(m_armMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    updateArmAngle();
  }

  public Command armDefaultCommand() {
    return run(() -> m_targetAngle = DEFAULT_POSITION);
  }

  // move the arm a desired amount
  public Command armRotateCommand(double targetAngle) {
    return run(() -> m_targetAngle = targetAngle);
  }

}