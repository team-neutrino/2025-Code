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

public class Wrist extends SubsystemBase {
  private SparkMax m_wristMotor = new SparkMax(WRIST, MotorType.kBrushless);
  private SparkMaxConfig m_wristConfig = new SparkMaxConfig();
  private double wristVoltage;
  private double lastAngle;
  private boolean hasCurrentSpiked;

  public Wrist() {
    m_wristConfig.smartCurrentLimit(WRIST_CURRENT_LIMIT);
    m_wristConfig.idleMode(IdleMode.kBrake);
    m_wristConfig.openLoopRampRate(0.25);
    m_wristMotor.configure(m_wristConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  /**
   * Moves the wrist to the given position.
   * <p>
   * Uses {@link #hasCurrentSpiked} to know whether or not to deny the request. If
   * the request is denied, the wrist will stay in the last position it was in.
   * 
   * @param angle The angle to move to.
   */
  public void moveToPosition(double angle) {
    updateCurrentSpike(angle);
    if (hasCurrentSpiked || (angle != WRIST_INTAKE_POS && angle != WRIST_SCORING_POS)) {
      wristVoltage = 0;
      return;
    }
    lastAngle = angle;
    wristVoltage = (angle == WRIST_INTAKE_POS ? WRIST_VOLTAGE : -WRIST_VOLTAGE);
  }

  public boolean voltageAgrees() {
    return Math.abs(wristVoltage - m_wristMotor.getOutputCurrent()) < 0.1;
  }

  public boolean isCurrentSpike() {
    return m_wristMotor.getOutputCurrent() > WRIST_CURRENT_LIMIT;
  }

  /**
   * Updates {@link #hasCurrentSpiked} using the following criteria:
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
    hasCurrentSpiked = !hasCurrentSpiked ? m_wristMotor.getOutputCurrent() > HARDSTOP_CURRENT_LIMIT
        : (requestedAngle == lastAngle);
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
        moveToPosition(WRIST_SCORING_POS);
      } else {
        moveToPosition(WRIST_INTAKE_POS);
      }
    });
  }

  public Command rotateWrist(double angle) {
    return run(() -> moveToPosition(angle));
  }

  @Override
  public void periodic() {
    m_wristMotor.setVoltage(wristVoltage);
  }

}
