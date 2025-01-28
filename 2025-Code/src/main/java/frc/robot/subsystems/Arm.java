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
import com.revrobotics.spark.config.SparkFlexConfigAccessor;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import static frc.robot.Constants.ArmConstants.*;

/**
 * Class that represents the arm subsystem on the robot.
 */
public class Arm extends SubsystemBase {
  /**
   * Arm motor
   */
  private SparkFlex m_armMotor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
  /**
   * Configuration object for the arm motor
   */
  private SparkFlexConfig m_armMotorConfig = new SparkFlexConfig();
  /**
   * Encoder for the arm motor
   */
  private AbsoluteEncoder m_armEncoder;
  /**
   * Closed loop controller for the arm motor
   */
  private SparkClosedLoopController m_armPidController;
  /**
   * Acessor for the arm motor
   */
  private SparkFlexConfigAccessor m_sparkFlexConfigAccessor;
  /**
   * Acessor for the arm configuration object
   */
  public ClosedLoopConfigAccessor m_armPidAccessor;
  /**
   * Target angle that the arm is going towards
   */
  private double m_targetAngle = 0;

  /**
   * Class constructor
   */
  public Arm() {
    initializeMotorControllers();
    m_sparkFlexConfigAccessor = m_armMotor.configAccessor;
    m_armPidAccessor = m_sparkFlexConfigAccessor.closedLoop;
  }

  /**
   * Gets the position os the arm motor
   * 
   * @return arm encoder position
   */
  public double getArmEncoderPosition() {
    return m_armEncoder.getPosition();
  }

  /**
   * Gets the target position that the arm is set to
   * 
   * @return m_targetAngle
   */
  public double getArmTargetPosition() {
    return m_targetAngle;
  }

  /**
   * Gets the voltage of the arm motor
   * 
   * @return arm motor bus voltage
   */
  public double getArmVoltage() {
    return m_armMotor.getBusVoltage();
  }

  /**
   * Sets up motor controllers to necessary configurations. Includes maxMotion,
   * current limits, and converstion factors.
   */
  private void initializeMotorControllers() {
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
    m_armPidController = m_armMotor.getClosedLoopController();

    m_armMotorConfig.closedLoop.maxMotion
        .maxVelocity(MAX_VELOCITY)
        .maxAcceleration(MAX_ACCELERATION)
        .allowedClosedLoopError(ALLOWED_ERROR);

    m_armMotor.configure(m_armMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Finds out if the arm is in limit
   * 
   * @return m_armLimit
   */
  public boolean isArmInLimit() {
    return true;
  }

  /**
   * Updates the angle that the arm is set to
   */
  public void updateArmAngle() {
    m_armPidController.setReference(m_targetAngle,
        SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardCalculation());
  }

  /**
   * Determines the necessary volts needed for the Feedforward. Used to pass into
   * closed loop controller
   * 
   * @return volts
   */
  public double feedForwardCalculation() {
    double currentAngle = getArmEncoderPosition();
    double volts = FFCONSTANT * Math.cos(currentAngle);
    return volts;
  }

  /**
   * Changes the PID settings of the closed loop controller
   * 
   * @param p Proportional
   * @param i Integral
   * @param d Derivative
   */
  public void changePID(double p, double i, double d) {
    m_armMotorConfig.closedLoop.pid(p, i, d);
    m_armMotor.configure(m_armMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    updateArmAngle();
  }

  /**
   * Gives a instance of the arm default command. Rotates the arm to the default
   * position
   * 
   * @return The rotate wrist command
   */
  public Command armDefaultCommand() {
    return run(() -> m_targetAngle = DEFAULT_POSITION);
  }

  /**
   * Gives a instance of the arm rotate command. Rotates the arm to the given
   * angle
   * 
   * @param targetAngle
   * @return The arm rotate command
   */
  public Command armRotateCommand(double targetAngle) {
    return run(() -> m_targetAngle = targetAngle);
  }
}