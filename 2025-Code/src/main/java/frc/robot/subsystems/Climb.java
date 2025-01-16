// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Climb extends SubsystemBase {
  private SparkFlex m_climbMotor1 = new SparkFlex(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
  private SparkFlex m_climbMotor2 = new SparkFlex(ClimbConstants.CLIMB_MOTOR_ID2, MotorType.kBrushless);
  private SparkFlex m_lockClimbMotor = new SparkFlex(ClimbConstants.CLIMB_MOTOR_ID3, MotorType.kBrushless);
  private SparkAbsoluteEncoder m_climbEncoder;
  private SparkLimitSwitch m_lockLimitSwitch = m_lockClimbMotor.getForwardLimitSwitch();

  private SparkFlexConfig m_climbMotorConfig = new SparkFlexConfig();
  private SparkFlexConfig m_lockClimbMotorConfig = new SparkFlexConfig();
  private SparkFlexConfig m_followMotorConfig = new SparkFlexConfig();

  private SparkClosedLoopController m_climbPID = m_climbMotor1.getClosedLoopController();

  private LimitSwitchConfig m_lockLimitSwitchConfig = new LimitSwitchConfig();

  private int m_targetAngle = 0;

  public Climb() {
    m_climbMotorConfig.smartCurrentLimit(CLIMB_CURRENT_LIMIT);
    m_climbMotorConfig.idleMode(IdleMode.kBrake);
    m_climbMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    m_climbMotorConfig.closedLoop.pid(1.0, 0.0, 0.0);
    m_climbMotorConfig.encoder.positionConversionFactor(360);

    m_lockClimbMotorConfig.smartCurrentLimit(ClimbConstants.LOCK_CURRENT_LIMIT);
    m_lockClimbMotorConfig.idleMode(IdleMode.kBrake);

    m_followMotorConfig.apply(m_climbMotorConfig);
    m_followMotorConfig.follow(m_climbMotor1, true);

    m_lockLimitSwitchConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

    m_climbMotor1.configure(m_climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_climbMotor2.configure(m_followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_lockClimbMotor.configure(m_lockClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_climbEncoder = m_climbMotor1.getAbsoluteEncoder();

  }

  private void lockClimb() {
    if (!m_lockLimitSwitch.isPressed()) {
      m_lockClimbMotor.set(ClimbConstants.LOCK_SPEED);
    } else {
      m_lockClimbMotor.stopMotor();
    }
  }

  private void setArmAngle(int angle) {
    m_climbPID.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardCalculation());
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

  public Command moveCLimbArmCommand(int angle) {
    return new RunCommand(() -> moveClimbArm(angle), this);
  }

  @Override
  public void periodic() {
    setArmAngle(m_targetAngle);
  }
}
