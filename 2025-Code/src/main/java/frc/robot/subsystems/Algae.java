// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

import static frc.robot.Constants.AlgaeConstants.*;

/**
 * Class that represents the algae subsystem on the robot.
 */
public class Algae extends SubsystemBase {

    private SparkMax m_motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    private SparkMaxConfig m_config = new SparkMaxConfig();
    private RelativeEncoder m_encoder;
    private double m_voltage;
    private Canandcolor m_colorSensor = new Canandcolor(COLOR_SENSOR);
    private CanandcolorSettings m_settings = new CanandcolorSettings();
    private Debouncer m_debouncer = new Debouncer(0.1, DebounceType.kFalling);

    public Algae() {
        m_encoder = m_motor.getEncoder();

        m_config.smartCurrentLimit(CURRENT_LIMIT);
        m_config.inverted(false);
        m_config.idleMode(IdleMode.kCoast);

        m_config.softLimit.forwardSoftLimitEnabled(false);
        m_config.softLimit.reverseSoftLimitEnabled(false);

        m_motor.configure(m_config, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        m_settings.setLampLEDBrightness(1);
        m_colorSensor.setSettings(m_settings);
    }

    private boolean withinProximity(double distance) {
        return m_colorSensor.getProximity() < distance;
    }

    public boolean hasAlgae() {
        return withinProximity(PROXIMITY);
    }

    /**
     * debounced the falling edge of hasAlgae for use with scoring commands
     */
    public boolean debouncedHasAlgae() {
        return m_debouncer.calculate(hasAlgae());
    }

    public double getAngularVelocity() {
        return m_encoder.getVelocity();
    }

    public double getMotorVoltage() {
        return m_voltage;
    }

    @Override
    public void periodic() {
        m_motor.set(m_voltage);
    }

    public Command algaeDefaultCommand() {
        return run(() -> {
            m_voltage = HOLD_PIECE_VOLTAGE;
        });
    }

    /**
     * Gives an instance of the run intake command., sets the intake voltage to the
     * speed provided.
     * 
     * @param speed speed algae is set to
     * @return The run intake command
     */
    public Command runIntake(double voltage) {
        return run(() -> m_voltage = voltage);
    }

}