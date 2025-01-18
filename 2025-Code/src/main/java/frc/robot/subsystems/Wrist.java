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
   * Moves the wrist to the given position. See helper method {@link #canMove()}
   * for the conditions by which the request is denied. If the request is denied,
   * the wrist will stay in the last position it was in
   * 
   * @param angle The angle to move to.
   */
  public void moveToPosition(double angle) {
    if (!canMove(angle)) {
      return;
    }
    hasCurrentSpiked = false;
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
   * Helper method for {@link #moveToPosition} that determines whether or not the
   * wrist can move given the requested angle to move to using the following
   * criteria:
   * <p>
   * 1.) The motor is not current spiking (running up against the hardstop)
   * <p>
   * 2.) The requested angle isn't the same as the last requested angle
   * <p>
   * 3.) The requested angle is either the intake or scoring position
   * 
   * @param requestedAngle The requested angle to move the wrist to.
   * @return Whether or not the wrist can move.
   */
  private boolean canMove(double requestedAngle) {
    if (!hasCurrentSpiked) {
      return true;
    }
    return requestedAngle != lastAngle && (requestedAngle == WRIST_INTAKE_POS || requestedAngle == WRIST_SCORING_POS);
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
