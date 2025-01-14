// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private SparkFlex m_motor1 = new SparkFlex(ElevatorConstants.MOTOR1, MotorType.kBrushless);
  private SparkFlex m_motor2 = new SparkFlex(ElevatorConstants.MOTOR2, MotorType.kBrushless);
  private SparkRelativeEncoder m_encoder;
  private SparkLimitSwitch m_highLimit;
  private SparkLimitSwitch m_lowLimit;
  private SparkClosedLoopController m_pid = m_motor1.getClosedLoopController();
  private SparkFlexConfig m_config = new SparkFlexConfig();
  private SparkFlexConfig m_followerConfig = new SparkFlexConfig();

  private double m_target = 0.0;

  public Elevator() {
    m_config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    m_config.encoder
        .positionConversionFactor(0)
        .velocityConversionFactor(0);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0);
    m_motor1.configure(m_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_followerConfig.follow(1);
    m_followerConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    m_followerConfig.encoder
        .positionConversionFactor(0)
        .velocityConversionFactor(0);
    m_followerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0);
    m_motor2.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setTargetHeight(double target) {
    m_target = target;
  }

  private void adjustElevator(double target) {
    m_pid.setReference(target, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardCalculation());
  }

  private double feedForwardCalculation() {
    return 0.0;
  }

  private void resetEncoder(double position) {
    m_encoder.setPosition(position);
  }

  private boolean isLowPosition() {
    return m_lowLimit.isPressed();
  }

  public boolean isHighPosition() {
    return m_highLimit.isPressed();
  }

  @Override
  public void periodic() {
    if (isLowPosition()) {
      resetEncoder(ElevatorConstants.LOW_POSITION);
    }
    if (isHighPosition()) {
      resetEncoder(ElevatorConstants.HIGH_POSITION);
    }
    adjustElevator(m_target);
  }

  @Override
  public void simulationPeriodic() {

  }
}
