
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

/**
 * Class that represents the claw subsystem on the robot.
 */
public class Claw extends SubsystemBase {

    /**
     * Grabber motor
     */
    private SparkMax m_grabber = new SparkMax(ClawConstants.LEFT_GRABBER, MotorType.kBrushless);
    /**
     * Follower motor to the grabber
     */
    private SparkMax m_follower = new SparkMax(ClawConstants.RIGHT_GRABBER, MotorType.kBrushless);
    /**
     * Configuration object for the grabber motor
     */
    private SparkMaxConfig m_grabberConfig = new SparkMaxConfig();
    /**
     * Configuration object for the follower motor
     */
    private SparkMaxConfig m_followerConfig = new SparkMaxConfig();
    /**
     * Encoder for the grabber motor
     */
    private RelativeEncoder m_grabberEncoder;
    /**
     * Encoder for the follower motor
     */
    private RelativeEncoder m_followerEncoder;
    /**
     * Voltage for the intake (controls power of the motor)
     */
    private double m_intakeVoltage;
    /**
     * Color sensor for the Grabber
     */
    private Canandcolor m_colorSensor = new Canandcolor(ClawConstants.COLOR_SENSOR);
    /**
     * Settings of the color sensor(used for lamp brightness)
     */
    private CanandcolorSettings m_settings = new CanandcolorSettings();

    /**
     * Class constructor
     */
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
        m_settings.setLampLEDBrightness(1);
        m_colorSensor.setSettings(m_settings);
    }

    private boolean withinProximity(double distance) {
        return m_colorSensor.getProximity() < distance;
    }

    private double getBlueToRed() {
        return m_colorSensor.getBlue() / m_colorSensor.getRed();
    }

    /**
     * Returns whether game piece in the claw is a algae
     * 
     * @return true if algae
     */
    public boolean isAlgae() {
        return withinProximity(0.15) && getBlueToRed() > 1.5;
    }

    /**
     * Returns whether game piece in the claw is a coral
     * 
     * @return true if coral
     */
    public boolean isCoral() {
        return withinProximity(0.15) && getBlueToRed() > 0.6 && getBlueToRed() < 1.15;
    }

    /**
     * Returns whether their is a game piece in the claw
     * 
     * @return if the claw has a game piece
     */
    public boolean hasGamePiece() {
        boolean ret = isCoral() && isAlgae() ? false : true;
        return ret && (isCoral() || isAlgae());
    }

    /**
     * Gets the velocity of the grabber
     * 
     * @return grabber velocity
     */
    public double getVelocityOfGrabber() {
        return m_grabberEncoder.getVelocity();
    }

    /**
     * Gets the velocity of the follower Motor
     * 
     * @return follower velocity
     */
    public double getVelocityOfGrabberFollower() {
        return m_followerEncoder.getVelocity();
    }

    /**
     * Gets intake volatge
     * 
     * @return intake voltage
     */
    public double getIntakeVoltage() {
        return m_intakeVoltage;
    }

    @Override
    public void periodic() {
        m_grabber.set(m_intakeVoltage);
        System.out.println(isAlgae());
        System.out.println(isCoral());
        System.out.println(m_colorSensor.getBlue());
        System.out.println(m_colorSensor.getRed());
    }

    /**
     * Gives an instance of the claw default command. Stops intake from running
     * 
     * @return The claw default command
     */
    public Command clawDefaultCommand() {
        return run(() -> m_intakeVoltage = 0);
    }

    /**
     * Gives an instance of the run intake command., sets the intake voltage to the speed provided.
     * 
     * @param speed speed claw is set to
     * @return The run intake command
     */
    public Command runIntake(double speed) {
        return run(() -> m_intakeVoltage = speed);
    }

}