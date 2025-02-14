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
import com.ctre.phoenix6.configs.MotionMagicConfigs;
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
  private SparkClosedLoopController m_pid = m_lockMotor.getClosedLoopController();

  private Servo m_lockRatchet = new Servo(RATCHET_PORT);

  private double m_targetPositionClimbArm;
  private double m_targetPositionLock;

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

    m_motionMagicConfig.MotionMagicCruiseVelocity = VELOCITY;
    m_motionMagicConfig.MotionMagicAcceleration = ACCELERATION;
    m_motionMagicConfig.MotionMagicJerk = JERK;

    // m_climbMotorConfig.withMotionMagic(m_motionMagicConfig);

    m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);

    m_followMotorConfig.CurrentLimits = m_currentLimitConfig;
    m_followMotor.setNeutralMode(NeutralModeValue.Brake);
    m_followMotor.getConfigurator().apply(m_followMotorConfig);
    m_followMotor.setControl(m_followRequest);

    m_lockMotorConfig.smartCurrentLimit(LOCK_CURRENT_LIMIT);
    m_lockMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(LOCK_kP, LOCK_kI, LOCK_kD);
    m_lockMotor.configure(m_lockMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void resetEncoderPosition() {
    m_climbMotor.setPosition(0);
    m_lockMotorEncoder.setPosition(0);
  }

  private void moveToPosition(double targetPosition) {
    PositionVoltage positionControl = new PositionVoltage(targetPosition);
    m_targetPositionClimbArm = targetPosition;
    m_climbMotor.setControl(positionControl);
  }

  /**
   * checks if climb arm is within a certain range of error
   */
  private boolean isTargetPosition() {
    return Math
        .abs(m_targetPositionClimbArm - m_climbMotor.getPosition().getValueAsDouble()) < CLIMB_MOTOR_POSITION_ERROR;
  }

  public Command engageRatchetCommand() {
    return run(() -> {
      m_lockRatchet.set(RATCHET_LOCK_POSITION);
    });
  }

  public Command disengageRatchetCommand() {
    return run(() -> {
      m_lockRatchet.set(RATCHET_UNLOCK_POSITION);
    });
  }

  public Command lockCommand(double targetPosition) {
    return run(() -> {
      m_targetPositionLock = targetPosition;
    });
  }

  public Command resetLockCommand(double targetPosition) {
    return run(() -> {
      m_targetPositionLock = targetPosition;
    });
  }

  public Command moveClimbArmCommand(double targetPosition) {
    return run(() -> {
      moveToPosition(targetPosition);
    });
    //.until(() -> isTargetPosition());
  }

  /**
   * only use when climb arm is in up position
   */
  public Command resetClimbArmCommand(double rotations) {
    return run(() -> {
      moveToPosition(rotations);
      m_climbMotor.setPosition(0);
    });
    // .until(() -> isTargetPosition());
  }

  public Command climbDefaultCommand() {
    return run(() -> {
      moveToPosition(CLIMB_DOWN_POSITION);
      m_lockRatchet.set(RATCHET_LOCK_POSITION);
      
    });
  }

  @Override
  public void periodic() {
    m_pid.setReference(m_targetPositionLock, ControlType.kPosition);
  }

  /* NETWORK TABLES */
  public double getMotorPosition() {
    return m_climbMotor.getPosition().getValueAsDouble();
  }

  public double getFollowerPosition() {
    return m_followMotor.getPosition().getValueAsDouble();
  }

  public double getTargetPosition() {
    return m_targetPositionClimbArm;
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

  public double getLockMotorCurrent() {
    return m_lockMotor.getOutputCurrent();
  }

  public void changePID(double p, double i, double d) {
    m_climbMotorConfig.Slot0.kP = p;
    m_climbMotorConfig.Slot0.kI = i;
    m_climbMotorConfig.Slot0.kD = d;

    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
  }

  public void changeLockPID(double p, double i, double d) {
    m_lockMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(p, i, d);

    m_lockMotor.configure(m_lockMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void changeMotionMagic(double velocity, double acceleration, double jerk) {
    m_motionMagicConfig.MotionMagicCruiseVelocity = velocity;
    m_motionMagicConfig.MotionMagicAcceleration = acceleration;
    m_motionMagicConfig.MotionMagicJerk = jerk;

    m_climbMotorConfig.withMotionMagic(m_motionMagicConfig);
    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
  }
}