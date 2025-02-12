// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Subsystem;

import static frc.robot.Constants.ElevatorConstants.*;

import java.time.Period;

import static frc.robot.Constants.ArmConstants.CORAL_STATION_POSITION;
import static frc.robot.Constants.ConfigSignals.*;

public class Elevator extends SubsystemBase {
  private SparkFlex m_motor1 = new SparkFlex(MOTOR1_ID, MotorType.kBrushless);
  private SparkFlex m_motor2 = new SparkFlex(MOTOR2_ID,
      MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_motor1.getEncoder();
  private SparkLimitSwitch m_lowLimit = m_motor1.getReverseLimitSwitch();
  private SparkClosedLoopController m_pid = m_motor1.getClosedLoopController();
  private SparkFlexConfig m_config = new SparkFlexConfig();
  private SparkFlexConfig m_followerConfig = new SparkFlexConfig();

  private double m_target = 0.0;
  private double m_FFConstant1 = STAGE_1_FF_VAL;
  private double m_FFConstant2 = STAGE_2_FF_VAL;

  public Elevator() {
    m_config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    m_config.encoder
        .positionConversionFactor(
            (1))
        .velocityConversionFactor(1);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(P_VAL, I_VAL, D_VAL);
    m_config.closedLoop.maxMotion
        .maxVelocity(MAX_VELOCITY)
        .maxAcceleration(MAX_ACCELERATION)
        .allowedClosedLoopError(ALLOWED_ERROR);
    m_config.signals.absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .appliedOutputPeriodMs(Period_MS_Fast)
        .busVoltagePeriodMs(Period_MS_Fast)
        .externalOrAltEncoderPositionAlwaysOn(false)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        .limitsPeriodMs(Period_MS_Fast)
        .motorTemperaturePeriodMs(Period_MS_Fast)
        .outputCurrentPeriodMs(Period_MS_Fast)
        .primaryEncoderPositionPeriodMs(Period_MS_Slow)
        .primaryEncoderVelocityPeriodMs(Period_MS_Slow);
    m_followerConfig.signals.absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .appliedOutputPeriodMs(Period_MS_Fast)
        .busVoltagePeriodMs(Period_MS_Fast)
        .externalOrAltEncoderPositionAlwaysOn(false)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        .limitsPeriodMs(Period_MS_Fast)
        .motorTemperaturePeriodMs(Period_MS_Fast)
        .outputCurrentPeriodMs(Period_MS_Fast)
        .primaryEncoderPositionPeriodMs(Period_MS_Slow)
        .primaryEncoderVelocityPeriodMs(Period_MS_Slow);
    m_config.smartCurrentLimit(CURRENT_LIMIT);
    m_motor1.configure(m_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_followerConfig.follow(MOTOR1_ID, true);
    m_followerConfig
        .idleMode(IdleMode.kBrake);
    m_followerConfig.smartCurrentLimit(CURRENT_LIMIT);
    m_motor2.configure(m_followerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  private void adjustElevator(double target) {
    m_pid.setReference(target, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedForwardCalculation());
  }

  private double feedForwardCalculation() {
    if (m_encoder.getPosition() < STAGE_ONE_UP) {
      return STAGE_1_FF_VAL;
    } else {
      return STAGE_2_FF_VAL;
    }
  }

  private void resetEncoder(double position) {
    m_encoder.setPosition(position);
  }

  public double getEncoderVelocity() {
    return m_encoder.getVelocity();
  }

  public double getEncoderPosition() {
    return m_encoder.getPosition();
  }

  public double getTargetPosition() {
    return m_target;
  }

  public boolean isLowPosition() {
    return m_lowLimit.isPressed();
  }

  public boolean isHighPosition() {
    return m_encoder.getPosition() > HIGH_POSITION;
  }

  public boolean elevatorReady() {
    if (m_target == DEFAULT || m_target == CORAL_STATION_POSITION || m_target == ALGAE_INTAKE) {
      return false;
    } else {
      return getEncoderPosition() >= m_target - 0.2;
    }
  }

  private double safeHeight(double targetHeight) {
    double safeTarget = targetHeight;
    if (!Subsystem.arm.isArmInLimit() && targetHeight < ARM_WILL_NOT_HIT_BASE_HEIGHT) {
      safeTarget = ARM_WILL_NOT_HIT_BASE_HEIGHT;
    }
    return safeTarget;
  }

  public Command elevatorDefaultCommand() {
    return run(() -> m_target = DEFAULT);
  }

  public Command moveElevatorCommand(double height) {
    return run(() -> m_target = height);
  }

  public void changePID(double p, double i, double d) {
    m_config.closedLoop.pid(p, i, d);
    m_motor1.configure(m_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void changeFF1(double newFF) {
    m_FFConstant1 = newFF;
  }

  public void changeFF2(double newFF) {
    m_FFConstant2 = newFF;
  }

  public void changeMaxMotion(double mv, double ma, double ae) {
    m_config.closedLoop.maxMotion.maxVelocity(mv).maxAcceleration(ma).allowedClosedLoopError(ae);
    m_motor1.configure(m_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    adjustElevator(safeHeight(m_target));
    if (isLowPosition()) {
      resetEncoder(LOW_POSITION);
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
