// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class Climb extends SubsystemBase {
  private SparkFlex m_climbMotor1 = new SparkFlex(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
  private SparkFlex m_climbMotor2 = new SparkFlex(ClimbConstants.CLIMB_MOTOR_ID2, MotorType.kBrushless);
  private SparkFlex m_holdClimbMotor = new SparkFlex(ClimbConstants.CLIMB_MOTOR_ID3, MotorType.kBrushless);
  private SparkAbsoluteEncoder m_climbEncoder;
  private DigitalInput m_lockLimitSwitch = new DigitalInput(ClimbConstants.LOCK_LIMIT_SWITCH);

  private SparkFlexConfig m_climbMotorConfig = new SparkFlexConfig();
  private SparkFlexConfig m_holdClimbMotorConfig = new SparkFlexConfig();
  private SparkFlexConfig m_followMotorConfig = new SparkFlexConfig();

  public Climb() {
    m_climbMotorConfig.smartCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT);
    m_climbMotorConfig.idleMode(IdleMode.kBrake);

    m_holdClimbMotorConfig.smartCurrentLimit(ClimbConstants.HOLD_CURRENT_LIMIT);
    m_holdClimbMotorConfig.idleMode(IdleMode.kBrake);

    m_followMotorConfig.apply(m_climbMotorConfig);
    m_followMotorConfig.follow(m_climbMotor1);

    m_climbMotor1.configure(m_climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_climbMotor2.configure(m_followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_holdClimbMotor.configure(m_holdClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_climbEncoder = m_climbMotor1.getAbsoluteEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
