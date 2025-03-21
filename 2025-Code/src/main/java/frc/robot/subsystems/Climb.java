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
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb extends SubsystemBase {
  private final CANBus m_CANBus = new CANBus("rio");

  private TalonFX m_climbMotor = new TalonFX(CLIMB_MOTOR_ID, m_CANBus);
  private TalonFX m_climbFollower = new TalonFX(FOLLOW_MOTOR_ID, m_CANBus);
  private TalonFXConfiguration m_climbMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_climbFollowerConfig = new TalonFXConfiguration();

  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private Follower m_followRequest = new Follower(CLIMB_MOTOR_ID, true);

  private Servo m_ratchetServo = new Servo(RATCHET_SERVO_PORT);

  private double m_targetPositionClimb;
  private double m_targetPositionRatchet;

  private boolean m_climbMotorOff;

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

    m_climbMotorConfig.Slot0.kP = CLIMB_kP;
    m_climbMotorConfig.Slot0.kI = CLIMB_kI;
    m_climbMotorConfig.Slot0.kD = CLIMB_kD;

    m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);

    m_climbFollowerConfig.CurrentLimits = m_currentLimitConfig;
    m_climbFollower.setNeutralMode(NeutralModeValue.Brake);
    m_climbFollower.getConfigurator().apply(m_climbFollowerConfig);
    m_climbFollower.setControl(m_followRequest);
  }

  private void resetEncoderPosition() {
    m_climbMotor.setPosition(0);
  }

  /**
   * checks if climb arm is in low position within a certain range of tolerance
   */
  private boolean isLowerPosition() {
    return Math
        .abs(LOWER_CLIMB_POSITION - m_climbMotor.getPosition().getValueAsDouble()) < CLIMB_POSITION_TOLERANCE;
  }

  public Command prepareClimbCommand() {
    return run(() -> {
      m_climbMotorOff = false;
      m_targetPositionClimb = m_climbMotor.getPosition().getValueAsDouble() - 0.5;
      m_targetPositionRatchet = RATCHET_UNLOCK_POSITION;
    });
  }

  public Command raiseClimbCommand() {
    return run(() -> {
      m_climbMotorOff = false;
      m_targetPositionRatchet = RATCHET_UNLOCK_POSITION;
      m_targetPositionClimb = RAISE_CLIMB_POSITION;
    });
  }

  public Command lowerClimbCommand() {
    return run(() -> {
      m_targetPositionRatchet = RATCHET_UNLOCK_POSITION;
      m_climbMotorOff = false;
      m_targetPositionClimb = LOWER_CLIMB_POSITION;
    }); //).until(() -> isLowerPosition()
  }

  public Command hasClimbCommand() {
    return run(() -> {
      m_targetPositionRatchet = RATCHET_LOCK_POSITION;
      m_climbMotorOff = true;
    });
  }

  public Command climbDefaultCommand() {
    return run(() -> {
      m_targetPositionRatchet = RATCHET_LOCK_POSITION;
      m_climbMotorOff = true;
    });
  }

  private void moveClimbArm(double targetPosition) {
    PositionVoltage positionControl = new PositionVoltage(targetPosition);
    m_climbMotor.setControl(positionControl);
  }

  @Override
  public void periodic() {
    if (m_climbMotorOff) {
      m_climbMotor.setVoltage(0);
    } else {
      moveClimbArm(m_targetPositionClimb);
    }
    m_ratchetServo.set(m_targetPositionRatchet);
  }

  /* NETWORK TABLES */
  public double getMotorPosition() {
    return m_climbMotor.getPosition().getValueAsDouble();
  }

  public double getFollowerPosition() {
    return m_climbFollower.getPosition().getValueAsDouble();
  }

  public double getTargetPosition() {
    return m_targetPositionClimb;
  }

  public double getClimbMotorCurrent() {
    return m_climbMotor.getSupplyCurrent().getValueAsDouble();
  }

  public double getClimbFollowerCurrent() {
    return m_climbFollower.getSupplyCurrent().getValueAsDouble();
  }

  public boolean getMotoroff() {
    return m_climbMotorOff;
  }

  public void changePID(double p, double i, double d) {
    m_climbMotorConfig.Slot0.kP = p;
    m_climbMotorConfig.Slot0.kI = i;
    m_climbMotorConfig.Slot0.kD = d;

    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
  }
}