// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

public class Climb extends SubsystemBase {
  private final CANBus m_CANBus = new CANBus("rio");

  private TalonFX m_climbMotor = new TalonFX(MAIN_MOTOR_ID, m_CANBus);
  private TalonFX m_followMotor = new TalonFX(FOLLOW_MOTOR_ID, m_CANBus);
  private TalonFXConfiguration m_climbMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_followMotorConfig = new TalonFXConfiguration();
  private MotionMagicConfigs m_motionMagicConfig = new MotionMagicConfigs();

  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private Follower m_followRequest = new Follower(MAIN_MOTOR_ID, true);

  private SparkMax m_lockMotor = new SparkMax(LOCK_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig m_lockMotorConfig = new SparkMaxConfig();
  private RelativeEncoder m_lockMotorEncoder = m_lockMotor.getEncoder();

  private double targetPosition;

  public Climb() {
    configureMotors();
    resetEncoderPosition();
  }

  private void configureMotors() {
    m_currentLimitConfig.withSupplyCurrentLimit(CLIMB_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(CLIMB_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_climbMotorConfig.CurrentLimits = m_currentLimitConfig;

    m_climbMotorConfig.Slot0.kP = kP;
    m_climbMotorConfig.Slot0.kI = kI;
    m_climbMotorConfig.Slot0.kD = kD;

    m_motionMagicConfig.MotionMagicCruiseVelocity = VELOCITY;
    m_motionMagicConfig.MotionMagicAcceleration = ACCELERATION;
    m_motionMagicConfig.MotionMagicJerk = JERK;

    m_climbMotorConfig.withMotionMagic(m_motionMagicConfig);

    m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);

    m_followMotor.setNeutralMode(NeutralModeValue.Brake);
    m_followMotor.getConfigurator().apply(m_followMotorConfig);
    m_followMotor.setControl(m_followRequest);

    m_lockMotorConfig.smartCurrentLimit(LOCK_CURRENT_LIMIT);

    m_lockMotor.configure(m_lockMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void resetEncoderPosition() {
    m_climbMotor.setPosition(0);
  }

  public void moveToPosition(double targetPosition) {
    PositionVoltage positionControl = new PositionVoltage(targetPosition);
    this.targetPosition = targetPosition;
    m_climbMotor.setControl(positionControl);
  }

  public Command lockCommand() {
    return runEnd(() -> {
      m_lockMotor.setVoltage(LOCK_VOLTAGE);
    }, () -> {
      m_lockMotor.setVoltage(0);
    }).until(() -> m_lockMotor.getOutputCurrent() > LOCK_CURRENT_THRESHOLD);
  }

  public Command moveClimbArmCommand(double targetPosition) {
    return run(() -> {
      moveToPosition(targetPosition);
    });
  }

  public Command climbDefaultCommand() {
    return run(() -> {
      moveToPosition(0);
    });
  }

  /* NETWORK TABLES */
  public double getMotorPosition() {
    return m_climbMotor.getPosition().getValueAsDouble();
  }

  public double getFollowerPosition() {
    return m_followMotor.getPosition().getValueAsDouble();
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public double getMotorVelocity() {
    return m_climbMotor.getVelocity().getValueAsDouble();
  }

  public double getFollowerVelocity() {
    return m_followMotor.getVelocity().getValueAsDouble();
  }

  public double getLockMotorVelocity() {
    return m_lockMotorEncoder.getVelocity();
  }

  public void changePID(double p, double i, double d) {
    m_climbMotorConfig.Slot0.kP = p;
    m_climbMotorConfig.Slot0.kI = i;
    m_climbMotorConfig.Slot0.kD = d;

    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
  }

  public void changeMotionMagic(double velocity, double acceleration, double jerk) {
    m_motionMagicConfig.MotionMagicCruiseVelocity = velocity;
    m_motionMagicConfig.MotionMagicAcceleration = acceleration;
    m_motionMagicConfig.MotionMagicJerk = jerk;

    m_climbMotorConfig.withMotionMagic(m_motionMagicConfig);
    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
  }

  @Override
  public void periodic() {

  }
}