// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Climb extends SubsystemBase {
  private TalonFX m_climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);
  private TalonFX m_followMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID2);
  private TalonFXConfiguration m_climbMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_followMotorConfig = new TalonFXConfiguration();

  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();

  // private SparkFlex m_climbMotor1 = new SparkFlex(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
  // private SparkFlex m_climbMotor2 = new SparkFlex(ClimbConstants.CLIMB_MOTOR_ID2, MotorType.kBrushless);
  private SparkMax m_lockClimbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID3, MotorType.kBrushless);
  private SparkAbsoluteEncoder m_climbEncoder;
  private SparkLimitSwitch m_lockLimitSwitch = m_lockClimbMotor.getForwardLimitSwitch();

  // private SparkFlexConfig m_climbMotorConfig = new SparkFlexConfig();
  private SparkMaxConfig m_lockClimbMotorConfig = new SparkMaxConfig();
  // private SparkFlexConfig m_followMotorConfig = new SparkFlexConfig();

  // private SparkClosedLoopController m_climbPID = m_climbMotor1.getClosedLoopController();

  private LimitSwitchConfig m_lockLimitSwitchConfig = new LimitSwitchConfig();

  private int m_targetAngle = 0;

  public Climb() {
    // m_climbMotorConfig.smartCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT);
    // m_climbMotorConfig.idleMode(IdleMode.kBrake);
    // m_climbMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // m_climbMotorConfig.closedLoop.pid(1.0, 0.0, 0.0);
    // m_climbMotorConfig.encoder.positionConversionFactor(360);

    // m_followMotorConfig.apply(m_climbMotorConfig);
    // m_followMotorConfig.follow(m_climbMotor1, true);

    m_lockLimitSwitchConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

    // m_climbMotor1.configure(m_climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // m_climbMotor2.configure(m_followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_lockClimbMotor.configure(m_lockClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // m_climbEncoder = m_climbMotor1.getAbsoluteEncoder();

  }

  private void configureMotors() {
    m_currentLimitConfig.withSupplyCurrentLowerLimit(ClimbConstants.LOCK_CURRENT_LIMIT);

    m_lockClimbMotorConfig.smartCurrentLimit(ClimbConstants.LOCK_CURRENT_LIMIT);
    m_lockClimbMotorConfig.idleMode(IdleMode.kCoast);

  }

  private void lockClimb() {
    if (!m_lockLimitSwitch.isPressed()) {
      m_lockClimbMotor.set(ClimbConstants.LOCK_SPEED);
    } else {
      m_lockClimbMotor.stopMotor();
    }
  }

  private void setArmAngle(int angle) {
    // m_climbPID.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardCalculation());
  }

  private double feedForwardCalculation() {
    return 0.0;
    // subject to change
  }

  private void moveClimbArm(int angle) {
    m_targetAngle = angle;
  }

  public Command lockCommand() {
    return new RunCommand(() -> lockClimb(), this);
  }

  public Command moveClimbArmCommand(int angle) {
    return new RunCommand(() -> moveClimbArm(angle), this);
  }

  @Override
  public void periodic() {
    setArmAngle(m_targetAngle);
  }
}
