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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;

public class Climb extends SubsystemBase {
  private final CANBus m_CANBus = new CANBus("rio");

  private TalonFX m_climbMotor = new TalonFX(CLIMB_MOTOR_ID, m_CANBus);
  private TalonFX m_climbFollower = new TalonFX(FOLLOW_MOTOR_ID, m_CANBus);
  private TalonFXConfiguration m_climbMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_climbFollowerConfig = new TalonFXConfiguration();

  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private Follower m_followRequest = new Follower(CLIMB_MOTOR_ID, true);

  private SparkMax m_grabMotor = new SparkMax(GRAB_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig m_grabMotorConfig = new SparkMaxConfig();
  private RelativeEncoder m_grabEncoder = m_grabMotor.getEncoder();
  private SparkClosedLoopController m_grabPid = m_grabMotor.getClosedLoopController();

  private Servo m_ratchetServo = new Servo(RATCHET_SERVO_PORT);

  private double m_targetPositionClimb;
  private double m_targetPositionGrab;
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

    m_grabMotorConfig.smartCurrentLimit(GRAB_CURRENT_LIMIT);
    m_grabMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(LOCK_kP, LOCK_kI, LOCK_kD);
    m_grabMotor.configure(m_grabMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void resetEncoderPosition() {
    m_climbMotor.setPosition(0);
    m_grabEncoder.setPosition(0);
  }

  /**
   * checks if climb arm is in low position within a certain range of tolerance
   */
  private boolean isLowerPosition() {
    return Math
        .abs(LOWER_CLIMB_POSITION - m_climbMotor.getPosition().getValueAsDouble()) < CLIMB_POSITION_TOLERANCE;
  }

  // /**
  // * checks if climb is safe to raise
  // */
  // private boolean isRaiseClimbSafe() {
  // return (Math.abs(GRANNY_GRABBER_POSITION - m_grabEncoder.getPosition()) <
  // GRABBER_POSITION_TOLERANCE) ||
  // Math.abs(LOCK_GRABBER_POSITION - m_grabEncoder.getPosition()) <
  // GRABBER_POSITION_TOLERANCE;
  // }

  /**
   * checks if grabbers are safe to move to lock position
   */
  private boolean isLockGrabSafe() {
    return Math.abs(UNLOCK_GRABBER_POSITION - m_grabEncoder.getPosition()) < GRABBER_POSITION_TOLERANCE;
  }

  /**
   * checks if climb is safe to lower
   */
  private boolean isLowerClimbSafe() {
    return Math.abs(RAISE_CLIMB_POSITION - m_climbMotor.getPosition().getValueAsDouble()) < CLIMB_POSITION_TOLERANCE &&
        Math.abs(LOCK_GRABBER_POSITION - m_grabEncoder.getPosition()) < GRABBER_LOCK_POSITION_TOLERANCE;
  }

  public Command resetGrabberCommand() {
    return run(() -> {
      m_targetPositionGrab = GRANNY_GRABBER_POSITION;
    });
  }

  public Command prepareClimbCommand() {
    return run(() -> {
      m_climbMotorOff = false;
      m_targetPositionClimb = m_climbMotor.getPosition().getValueAsDouble() - 0.5;
      m_targetPositionRatchet = RATCHET_UNLOCK_POSITION;
      m_targetPositionGrab = UNLOCK_GRABBER_POSITION;
    });
    // .onlyIf(() -> isRaiseClimbSafe());
  }

  public Command raiseClimbCommand() {
    return run(() -> {
      m_climbMotorOff = false;
      m_targetPositionRatchet = RATCHET_UNLOCK_POSITION;
      m_targetPositionGrab = UNLOCK_GRABBER_POSITION;
      m_targetPositionClimb = RAISE_CLIMB_POSITION;
    });
  }

  public Command lockGrabberCommand() {
    return run(() -> {
      m_targetPositionGrab = LOCK_GRABBER_POSITION;
    }).onlyIf(() -> isLockGrabSafe());
  }

  public Command lowerClimbCommand() {
    return run(() -> {
      m_targetPositionGrab = LOCK_GRABBER_POSITION;
      m_targetPositionRatchet = RATCHET_UNLOCK_POSITION;
      m_climbMotorOff = false;
      m_targetPositionClimb = LOWER_CLIMB_POSITION;
    }).onlyIf(() -> isLowerClimbSafe()).until(() -> isLowerPosition());
  }

  // public Command lockRatchetCommand() {
  // return run(() -> m_targetPositionRatchet = RATCHET_LOCK_POSITION);
  // }

  // public Command unlockRatchetCommand() {
  // return run(() -> m_targetPositionRatchet = RATCHET_UNLOCK_POSITION);
  // }

  public Command hasClimbCommand() {
    return run(() -> {
      m_targetPositionGrab = LOCK_GRABBER_POSITION;
      m_targetPositionRatchet = RATCHET_LOCK_POSITION;
      m_climbMotorOff = true;
    });
  }

  public Command climbDefaultCommand() {
    return run(() -> {
      m_targetPositionGrab = GRANNY_GRABBER_POSITION;
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
    m_grabPid.setReference(m_targetPositionGrab, ControlType.kPosition);
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

  public double getGrabMotorCurrent() {
    return m_grabMotor.getOutputCurrent();
  }

  public double getGrabMotorPosition() {
    return m_grabEncoder.getPosition();
  }

  public double getGrabMotorTargetPosition() {
    return m_targetPositionGrab;
  }

  // public boolean getIsRaiseClimbSafe() {
  // return isRaiseClimbSafe();
  // }

  public boolean getIsLockGrabSafe() {
    return isLockGrabSafe();
  }

  public boolean getIsLowerClimbSafe() {
    return isLowerClimbSafe();
  }

  public void changePID(double p, double i, double d) {
    m_climbMotorConfig.Slot0.kP = p;
    m_climbMotorConfig.Slot0.kI = i;
    m_climbMotorConfig.Slot0.kD = d;

    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
  }

  public void changeLockPID(double p, double i, double d) {
    m_grabMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(p, i, d);

    m_grabMotor.configure(m_grabMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}