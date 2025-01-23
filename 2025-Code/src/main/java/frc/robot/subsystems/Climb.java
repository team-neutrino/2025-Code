// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climb extends SubsystemBase {
  private final CANBus m_CANBus = new CANBus("rio");

  private TalonFX m_climbMotor = new TalonFX(CLIMB_MOTOR_ID, m_CANBus);
  private TalonFX m_followMotor = new TalonFX(CLIMB_MOTOR_ID2, m_CANBus);
  private TalonFXConfiguration m_climbMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_followMotorConfig = new TalonFXConfiguration();
  m_climbMotorConfig.Feedback = FeedbackSensorSourceValue.RotorSensor;


  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private Follower m_followRequest = new Follower(CLIMB_MOTOR_ID, true);

  private SparkMax m_lockClimbMotor = new SparkMax(CLIMB_MOTOR_ID3, MotorType.kBrushless);
  private SparkMaxConfig m_lockClimbMotorConfig = new SparkMaxConfig();

  private SparkLimitSwitch m_lockLimitSwitch = m_lockClimbMotor.getForwardLimitSwitch();

  private Servo m_climbRatchet = new Servo(CLIMB_RATCHET_PORT);
  private Servo m_lockRatchet = new Servo(LOCK_RATCHET_PORT);

  public Climb() {
    configureMotors();
  }

  private void configureMotors() {
    m_currentLimitConfig.withSupplyCurrentLimit(CLIMB_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(CLIMB_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_climbMotorConfig.CurrentLimits = m_currentLimitConfig;
    m_climbMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
    m_climbMotor.getConfigurator().setPosition(0);

    m_followMotor.setNeutralMode(NeutralModeValue.Brake);
    m_followMotor.getConfigurator().apply(m_followMotorConfig);
    m_followMotor.setControl(m_followRequest);

    m_lockClimbMotorConfig.smartCurrentLimit(LOCK_CURRENT_LIMIT);
    m_lockClimbMotorConfig.idleMode(IdleMode.kCoast);

    m_lockClimbMotor.configure(m_lockClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void lockClimb() {
    if (!m_lockLimitSwitch.isPressed()) {
      m_lockClimbMotor.set(LOCK_SPEED);
    } else {
      m_lockClimbMotor.stopMotor();
    }
  }

  private void runMotorByTicks(double ticks) {
    PositionVoltage positionControl = new PositionVoltage(ticks);
    m_climbMotor.setControl(positionControl);
  }

  public void engageLockRatchet() {
    m_lockRatchet.set(0);
    // subject to change
  }

  public void engageClimbRatchet() {
    m_climbRatchet.set(0);
    // subject to change
  }

  public void disengageLockRatchet() {
    m_lockRatchet.set(1);
    // subject to change
  }

  public void disengageClimbRatchet() {
    m_climbRatchet.set(1);
    // subject to change
  }

  public Command climbDefaultCommand() {
    return run(() -> {
      engageClimbRatchet();
      disengageLockRatchet();
      // does position need to be specified?
    });
  }

  public Command raiseClimbArmCommand(double ticks) {
    return run(() -> {
      disengageClimbRatchet();
      runMotorByTicks(ticks);
    });
  }

  public Command lowerClimbArmCommand(double ticks) {
    return run(() -> {
      runMotorByTicks(ticks);
      engageClimbRatchet();
    });
  }

  public Command lockCommand() {
    return run(() -> {
      lockClimb();
      engageLockRatchet();
    });
  }

  @Override
  public void periodic() {

  }
}