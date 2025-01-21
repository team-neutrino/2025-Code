// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climb extends SubsystemBase {
  // private final CANBus m_CANBus = new CANBus("rio");

  private TalonFX m_climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);
  private TalonFX m_followMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID2);
  private TalonFXConfiguration m_climbMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_followMotorConfig = new TalonFXConfiguration();

  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private Follower m_followRequest = new Follower(ClimbConstants.CLIMB_MOTOR_ID, true);

  private SparkMax m_lockClimbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID3, MotorType.kBrushless);
  private SparkMaxConfig m_lockClimbMotorConfig = new SparkMaxConfig();

  private SparkLimitSwitch m_lockLimitSwitch = m_lockClimbMotor.getForwardLimitSwitch();
  private LimitSwitchConfig m_lockLimitSwitchConfig = new LimitSwitchConfig();

  private Servo m_climbRatchet = new Servo(ClimbConstants.CLIMB_RATCHET_PORT);
  private Servo m_lockRatchet = new Servo(ClimbConstants.LOCK_RATCHET_PORT);

  // private CANcoder m_encoder = new CANcoder(ClimbConstants.CLIMB_ENCODER_ID,
  // m_CANBus);

  // private SparkFlex m_climbMotor = new
  // SparkFlex(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
  // private SparkFlex m_followMotor = new
  // SparkFlex(ClimbConstants.CLIMB_MOTOR_ID2, MotorType.kBrushless);

  // private SparkFlexConfig m_climbMotorConfig = new SparkFlexConfig();
  // private SparkFlexConfig m_followMotorConfig = new SparkFlexConfig();

  // private SparkClosedLoopController m_climbPID =
  // m_climbMotor1.getClosedLoopController();

  public Climb() {
    m_lockLimitSwitchConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

    configureMotors();

    // m_climbMotorConfig.smartCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT);
    // m_climbMotorConfig.idleMode(IdleMode.kBrake);
    // m_climbMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // m_climbMotorConfig.closedLoop.pid(1.0, 0.0, 0.0);
    // m_climbMotorConfig.encoder.positionConversionFactor(360);

    // m_followMotorConfig.apply(m_climbMotorConfig);
    // m_followMotorConfig.follow(m_climbMotor1, true);

    // m_climbMotor.configure(m_climbMotorConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    // m_followMotor.configure(m_followMotorConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    // m_climbEncoder = m_climbMotor1.getAbsoluteEncoder();

  }

  private void configureMotors() {
    m_currentLimitConfig.withSupplyCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_climbMotorConfig.CurrentLimits = m_currentLimitConfig;

    m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
    m_climbMotor.getConfigurator().apply(m_climbMotorConfig);
    m_climbMotor.getConfigurator().setPosition(0);

    m_followMotor.setNeutralMode(NeutralModeValue.Brake);
    m_followMotor.getConfigurator().apply(m_followMotorConfig);
    m_followMotor.setControl(m_followRequest);

    m_lockClimbMotorConfig.smartCurrentLimit(ClimbConstants.LOCK_CURRENT_LIMIT);
    m_lockClimbMotorConfig.idleMode(IdleMode.kCoast);

    m_lockClimbMotor.configure(m_lockClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // private void configurePID() {
  // m_PIDConfig.kP = 1.0;
  // m_PIDConfig.kI = 0.0;
  // m_PIDConfig.kD = 0.0;
  // // subject to change
  // m_climbMotorConfig.Slot0 = m_PIDConfig;
  // }

  private void lockClimb() {
    if (!m_lockLimitSwitch.isPressed()) {
      m_lockClimbMotor.set(ClimbConstants.LOCK_SPEED);
    } else {
      m_lockClimbMotor.stopMotor();
    }
  }

  public void runMotorByTicks(double ticks) {
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

  // private void setArmAngle(int angle) {
  // // m_climbPID.setReference(angle, ControlType.kPosition,
  // ClosedLoopSlot.kSlot0,
  // // feedForwardCalculation());
  // }

  // private void moveClimbArm(int angle) {
  // m_targetAngle = angle;
  // }

  public Command lockCommand() {
    return new RunCommand(() -> {
      lockClimb();
      engageLockRatchet();
    }, this);
  }

  public Command raiseClimbArmCommand(double ticks) {
    return new RunCommand(() -> runMotorByTicks(ticks), this);
  }

  public Command lowerClimbArmCommand(double ticks) {
    return new RunCommand(() -> {
      runMotorByTicks(ticks);
      engageClimbRatchet();
    }, this);
  }

  @Override
  public void periodic() {
    // setArmAngle(m_targetAngle);
  }
}
