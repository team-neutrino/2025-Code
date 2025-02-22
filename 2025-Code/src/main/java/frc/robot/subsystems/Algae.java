// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.Constants.CoralConstants.COLOR_SENSOR;
import static frc.robot.Constants.CoralConstants.HOLD_PIECE_VOLTAGE;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Coral;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {

  private SparkMax m_motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig m_motorConfig = new SparkMaxConfig();
  private RelativeEncoder m_encoder;
  private double m_motorVoltage;

  private CanandcolorSettings m_settings = new CanandcolorSettings();
  private Canandcolor m_colorSensor = new Canandcolor(COLOR_SENSOR);

  /** Creates a new Grabber. */
  public Algae() {
    initializeMotorControllers();
  }

  public void initializeMotorControllers() {
    m_encoder = m_motor.getEncoder();

    m_motorConfig.smartCurrentLimit(CURRENT_LIMIT);
    m_motorConfig.inverted(false);
    m_motorConfig.idleMode(IdleMode.kCoast);

    m_motorConfig.softLimit.forwardSoftLimitEnabled(false);
    m_motorConfig.softLimit.reverseSoftLimitEnabled(false);

    m_motor.configure(m_motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    m_settings.setLampLEDBrightness(1);
    m_colorSensor.setSettings(m_settings);
  }

  public double getAngularVelocity() {
    return m_encoder.getVelocity();
  }

  public double getMotorVoltage() {
    return m_motorVoltage;
  }

  public void setMotorVoltage(double voltage) {
    m_motorVoltage = voltage;
  }

  public Command runIntake(double speed) {
    return run(() -> setMotorVoltage(speed));
  }

  @Override
  public void periodic() {
    m_motor.set(m_motorVoltage);
    // This method will be called once per scheduler run
  }
}
