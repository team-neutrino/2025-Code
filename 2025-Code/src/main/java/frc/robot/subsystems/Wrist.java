// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Subsystem;

import static frc.robot.Constants.WristConstants.*;

/**
 * Class that represents the wrist subsystem on the robot.
 */
public class Wrist extends SubsystemBase {
  /**
   * Wrist motor
   */
  private SparkMax m_wristMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
  /**
   * Configuration object for the wrist motor
   */
  private SparkMaxConfig m_wristConfig = new SparkMaxConfig();
  /**
   * Voltage for the intake (controls power of the motor)
   */
  private double m_wristVoltage;
  /*
   * Last recorded angle that is passed into the wrist
   */
  private double m_lastAngle;
  /**
   * Whether the current has spiked
   */
  private boolean m_hasCurrentSpiked;

  /**
   * Class constructor
   */
  public Wrist() {
    m_wristConfig.smartCurrentLimit(CURRENT_LIMIT);
    m_wristConfig.idleMode(IdleMode.kBrake);
    m_wristConfig.openLoopRampRate(RAMP_RATE);
    m_wristMotor.configure(m_wristConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  /**
   * Moves the wrist to the given position.
   * <p>
   * Uses {@link #m_hasCurrentSpiked} to know whether or not to deny the request.
   * If
   * the request is denied, the wrist will stay in the last position it was in.
   * 
   * @param angle The angle to move to.
   */
  public void moveToPosition(double angle) {
    updateCurrentSpike(angle);
    if (m_hasCurrentSpiked || (angle != INTAKE_POS && angle != SCORING_POS)) {
      m_wristVoltage = 0;
      return;
    }
    m_lastAngle = angle;
    m_wristVoltage = (angle == INTAKE_POS ? VOLTAGE : -VOLTAGE);
  }

  /**
   * Determines if their is a current spike
   * 
   * @return if current spike
   */
  public boolean isCurrentSpike() {
    return m_wristMotor.getOutputCurrent() > CURRENT_LIMIT;
  }

  /**
   * Returns if their has been a current spike in the last passed angle.
   * 
   * @return m_hasCurrentSpiked
   */
  public boolean hasCurrentSpike() {
    return m_hasCurrentSpiked;
  }

  /**
   * Returns the wrist motor's voltage
   * 
   * @return m_wristVoltage
   */
  public double getWristVoltage() {
    return m_wristVoltage;
  }

  /**
   * Returns the last angle the wrist is put to
   * 
   * @return m_lastAngle
   */
  public double getLastAngle() {
    return m_lastAngle;
  }

  /**
   * Updates {@link #m_hasCurrentSpiked} using the following criteria:
   * <p>
   * If there hasn't been a current spike yet, the wrist was not at either
   * hardstop at the time of the last check. So, check if the current is too high
   * to determine if it has now run into the hardstop. If there has already been a
   * current spike, the wrist is at a hardstop, so if the angle being requested is
   * the same as the last one then the wrist is already there.
   * 
   * @param requestedAngle The requested angle to move the wrist to.
   * @return Whether or not the wrist can move.
   */
  private void updateCurrentSpike(double requestedAngle) {
    m_hasCurrentSpiked = !m_hasCurrentSpiked ? m_wristMotor.getOutputCurrent() > CURRENT_LIMIT
        : (requestedAngle == m_lastAngle);
  }

  /**
   * Gives an instance of the wrist default command.
   * <p>
   * This command uses the value of the claw beam break to determine whether or
   * not we have a game piece; if we have a game piece it moves the wrist to
   * scoring position and otherwise moves it to the intake position.
   * 
   * @return The wrist default command.
   */
  public Command wristDefaultCommand() {
    return run(() -> {
      if (Subsystem.claw.hasGamePiece()) {
        moveToPosition(SCORING_POS);
      } else {
        moveToPosition(INTAKE_POS);
      }
    });
  }
  /**
   * Gives a instance of the rotate wrist command. Rotates the wrist to the given angle
   * @return The rotate wrist command
   */
  public Command rotateWrist(double angle) {
    return run(() -> moveToPosition(angle));
  }

  @Override
  public void periodic() {
    m_wristMotor.setVoltage(m_wristVoltage);
  }

}
