// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.Subsystem;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.CANRateConstants.*;

@Logged
public class Elevator extends SubsystemBase {
  private SparkFlex m_motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
  private SparkFlex m_follower = new SparkFlex(FOLLOWER_ID, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_motor.getEncoder();
  private SparkLimitSwitch m_lowLimit = m_motor.getReverseLimitSwitch();
  private SparkClosedLoopController m_pid = m_motor.getClosedLoopController();
  private SparkFlexConfig m_config = new SparkFlexConfig();
  private SparkFlexConfig m_followerConfig = new SparkFlexConfig();

  private double m_targetHeight = BOTTOM_POSITION;
  private double m_FFStage1 = STAGE_1_FF;
  private double m_FFStage2 = STAGE_2_FF;

  public Elevator() {
    m_config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    m_config.encoder
        .positionConversionFactor((1))
        .velocityConversionFactor(1);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(P_VAL, I_VAL, D_VAL, ClosedLoopSlot.kSlot0)
        .pid(P_VAL_ALGAE, I_VAL_ALGAE, D_VAL_ALGAE, ClosedLoopSlot.kSlot1);
    m_config.closedLoop.maxMotion
        .maxVelocity(MAX_VELOCITY)
        .maxAcceleration(MAX_ACCELERATION)
        .allowedClosedLoopError(ALLOWED_ERROR);
    m_config.signals.absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .appliedOutputPeriodMs(FASTEST_5MS)
        .busVoltagePeriodMs(FASTEST_5MS)
        .externalOrAltEncoderPositionAlwaysOn(false)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        .limitsPeriodMs(FASTEST_5MS)
        .motorTemperaturePeriodMs(FASTEST_5MS)
        .outputCurrentPeriodMs(FASTEST_5MS)
        .primaryEncoderPositionPeriodMs(FAST_10MS)
        .primaryEncoderVelocityPeriodMs(FAST_10MS);
    m_followerConfig.signals.absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .appliedOutputPeriodMs(FASTEST_5MS)
        .busVoltagePeriodMs(FASTEST_5MS)
        .externalOrAltEncoderPositionAlwaysOn(false)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        .limitsPeriodMs(FASTEST_5MS)
        .motorTemperaturePeriodMs(FASTEST_5MS)
        .outputCurrentPeriodMs(FASTEST_5MS)
        .primaryEncoderPositionPeriodMs(FAST_10MS)
        .primaryEncoderVelocityPeriodMs(FAST_10MS);
    m_config.smartCurrentLimit(CURRENT_LIMIT);
    m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_followerConfig.follow(MOTOR_ID, true);
    m_followerConfig.idleMode(IdleMode.kBrake);
    m_followerConfig.smartCurrentLimit(CURRENT_LIMIT);
    m_follower.configure(m_followerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private void adjustElevator(double target) {
    if (Subsystem.algae.debouncedHasAlgae()) {
      m_pid.setReference(target, ControlType.kPosition, ClosedLoopSlot.kSlot1,
          feedForwardCalculation());
    } else {
      m_pid.setReference(target, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardCalculation());
    }
  }

  private double feedForwardCalculation() {
    if (m_encoder.getPosition() < STAGE_ONE_UP) {
      return m_FFStage1;
    } else {
      return m_FFStage2;
    }
  }

  private void resetEncoder(double position) {
    m_encoder.setPosition(position);
  }

  public double getBusVoltage() {
    return m_motor.getBusVoltage();
  }

  public double getOutputCurrent() {
    return m_motor.getOutputCurrent();
  }

  public int getFaultRawBits() {
    return m_motor.getStickyFaults().rawBits;
  }

  public boolean getGateDriver() {
    return m_motor.getStickyFaults().gateDriver;
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public double getHeight() {
    return m_encoder.getPosition();
  }

  public double getTargetHeight() {
    return m_targetHeight;
  }

  public boolean isAtBottom() {
    return m_lowLimit.isPressed();
  }

  private boolean atTargetHeight() {
    return Math.abs(getHeight() - m_targetHeight) <= HEIGHT_TOLERANCE;
  }

  public boolean readyToScore() {
    return atTargetHeight() && !(m_targetHeight == DEFAULT_NO_CORAL || m_targetHeight == CORAL_INTAKE
        || m_targetHeight == DEFAULT_WITH_CORAL);
  }

  private double safeHeight(double targetHeight) {
    double safeTarget = targetHeight;
    double safeHeight = Subsystem.algae.hasAlgae() ? SAFE_HEIGHT_ALGAE : SAFE_HEIGHT_NO_ALGAE;

    if ((Subsystem.arm.getAngle() < 90 || Subsystem.arm.getAngle() > 280 || Subsystem.arm.getTargetAngle() < 90
        || Subsystem.arm.getTargetAngle() > 280) && (getTargetHeight() < safeHeight)) {
      safeTarget = safeHeight;
    }

    else if ((((Subsystem.arm.getTargetAngle() > 180 && Subsystem.arm.getAngle() < 180)
        || (Subsystem.arm.getTargetAngle() < 180 && Subsystem.arm.getAngle() > 180))
        || (Subsystem.arm.getAngle() > ArmConstants.DEFAULT_POSITION + ArmConstants.DRIVING_ANGLE_TOLERANCE
            && Subsystem.arm.getAngle() < ArmConstants.DEFAULT_NO_GP - ArmConstants.DRIVING_ANGLE_TOLERANCE))
        && getTargetHeight() < safeHeight) {
      safeTarget = safeHeight;
    }
    return safeTarget;
  }

  public void changePID(double p, double i, double d) {
    m_config.closedLoop.pid(p, i, d);
    m_motor.configure(m_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void changeFF1(double newFF) {
    m_FFStage1 = newFF;
  }

  public void changeFF2(double newFF) {
    m_FFStage2 = newFF;
  }

  public void changeMaxMotion(double mv, double ma, double ae) {
    m_config.closedLoop.maxMotion.maxVelocity(mv).maxAcceleration(ma).allowedClosedLoopError(ae);
    m_motor.configure(m_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    adjustElevator(safeHeight(m_targetHeight));
    if (isAtBottom()) {
      resetEncoder(BOTTOM_POSITION);
    }
  }

  @Override
  public void simulationPeriodic() {

  }

  public Command elevatorDefaultCommand() {
    return run(() -> {
      if (Subsystem.coral.debouncedHasCoral() && !Subsystem.algae.debouncedHasAlgae()) {
        m_targetHeight = DEFAULT_WITH_CORAL;
      } else {
        m_targetHeight = DEFAULT_NO_CORAL;
      }
    });
  }

  public Command moveElevatorCommand(double height) {
    return run(() -> m_targetHeight = height);
  }
}
