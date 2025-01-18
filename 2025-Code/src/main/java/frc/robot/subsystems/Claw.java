
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.ColorData;

public class Claw extends SubsystemBase {

    private SparkMax m_grabber = new SparkMax(ClawConstants.LEFT_GRABBER, MotorType.kBrushless);
    private SparkMax m_follower = new SparkMax(ClawConstants.RIGHT_GRABBER, MotorType.kBrushless);

    private SparkMaxConfig m_grabberConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followerConfig = new SparkMaxConfig();

    private RelativeEncoder m_grabberEncoder;
    private RelativeEncoder m_followerEncoder;

    private double m_intakeVoltage;
    private DigitalInput m_intakeBeamBreak = new DigitalInput(ClawConstants.INTAKE_MOTOR_BEAMBREAK);
    Canandcolor colorSensor = new Canandcolor(ClawConstants.COLOR_SENSOR);

    public Claw() {
        m_grabberEncoder = m_grabber.getEncoder();
        m_followerEncoder = m_follower.getEncoder();

        m_grabberConfig.smartCurrentLimit(Constants.ClawConstants.GRABBER_CURRENT_LIMIT);
        m_grabberConfig.inverted(false);
        m_grabberConfig.idleMode(IdleMode.kCoast);

        m_grabberConfig.softLimit.forwardSoftLimitEnabled(false);
        m_grabberConfig.softLimit.reverseSoftLimitEnabled(false);
        m_followerConfig.apply(m_grabberConfig);
        m_followerConfig.follow(m_grabber);
        m_grabber.configure(m_grabberConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        m_follower.configure(m_followerConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    public boolean isAlgae() {
        ColorData colorData = colorSensor.getColor();
        return colorData.blue() > 100;
    }

    public boolean isCoral() {
        ColorData colorData = colorSensor.getColor();
        return colorData.blue() > 240 && colorData.red() > 240 && colorData.green() > 240;
    }

    public double getVelocityOfGrabber() {
        return m_grabberEncoder.getVelocity();
    }

    public double getVelocityOfGrabberFollower() {
        return m_followerEncoder.getVelocity();
    }

    public double getIntakeVoltage() {
        return m_intakeVoltage;
    }

    public boolean hasGamePiece() {
        return !m_intakeBeamBreak.get();
    }

    @Override
    public void periodic() {
        m_grabber.set(m_intakeVoltage);
    }

    public Command clawDefaultCommand() {
        return run(() -> m_intakeVoltage = 0);
    }

    public Command runIntake(double speed) {
        return run(() -> m_intakeVoltage = speed);
    }

}