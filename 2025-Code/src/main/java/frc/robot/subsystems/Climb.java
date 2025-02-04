// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.ClimbConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

public class Climb extends SubsystemBase {
  private final CANBus m_CANBus = new CANBus("rio");

  private TalonFX m_climbMotor = new TalonFX(MAIN_MOTOR_ID, m_CANBus);
  private TalonFX m_followMotor = new TalonFX(FOLLOW_MOTOR_ID, m_CANBus);
  private TalonFXConfiguration m_climbMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_followMotorConfig = new TalonFXConfiguration();

  private final CurrentLimitsConfigs m_currentLimitConfig = new CurrentLimitsConfigs();
  private Follower m_followRequest = new Follower(MAIN_MOTOR_ID, true);

  private SparkMax m_lockMotor = new SparkMax(LOCK_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig m_lockMotorConfig = new SparkMaxConfig();

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
    
    public Command moveToPositionCommand(double targetRotations) {
        return new RunCommand(() -> {
            PositionVoltage positionControl = new PositionVoltage(targetRotations);
            m_climbMotor.setControl(positionControl);
        }, this);
    }













  public void climbUp() {
    m_climbMotor.setVoltage(CLIMB_UP_VOLTAGE);
    // subject to change!!!!!!
  }

  public void climbDown() {
    m_climbMotor.setVoltage(CLIMB_DOWN_VOLTAGE);
    // subject to change!!!!!!
  }

  public Command lockCommand() {
    return runEnd(() -> {
      m_lockMotor.setVoltage(LOCK_VOLTAGE);
    }, () -> {
      m_lockMotor.setVoltage(0); 
    }).until(() -> m_lockMotor.getOutputCurrent() > LOCK_CURRENT_THRESHOLD);
  }

  public Command raiseClimbArmCommand() {
    return run(() -> {
      climbUp();
    });
  }

  public Command lowerClimbArmCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_climbMotor.setVoltage(12), this),
        new WaitCommand(3.0),
        new InstantCommand(() -> m_climbMotor.setVoltage(0), this));
  }

  public Command climbDefaultCommand() {
    return run(() -> {
      m_climbMotor.setVoltage(0);
    });
  }

  @Override
  public void periodic() {

  }
}