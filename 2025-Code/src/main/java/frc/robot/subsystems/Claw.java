// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {

    // grabber is the intake
    private SparkMax m_grabber = new SparkMax(ClawConstants.LEFTGRABBER, MotorType.kBrushless);
    private SparkMax m_follower = new SparkMax(ClawConstants.RIGHTGRABBER, MotorType.kBrushless);

    private SparkMaxConfig m_grabberConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followerConfig = new SparkMaxConfig();

    private RelativeEncoder m_grabberEncoder;
    private RelativeEncoder m_followerEncoder;

    private double clawVoltage;
    private boolean isBroken;
    private DigitalInput m_intakeBeamBreak = new DigitalInput(ClawConstants.INTAKE_MOTOR_BEAMBREAK);

    /** Creates a new ExampleSubsystem. */
    public Claw() {
        m_grabberEncoder = m_grabber.getEncoder();
        m_followerEncoder = m_follower.getEncoder();

        // space for current limits
        m_grabberConfig.smartCurrentLimit(Constants.ClawConstants.GRABBERCURRENTLIMIT);
        m_grabberConfig.inverted(false);
        m_grabberConfig.idleMode(IdleMode.kCoast);

        m_grabberConfig.softLimit.forwardSoftLimitEnabled(false);
        m_grabberConfig.softLimit.reverseSoftLimitEnabled(false);
        m_followerConfig.apply(m_grabberConfig);
        m_followerConfig.follow(m_grabber);
        m_grabber.configure(m_grabberConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        m_grabber.configure(m_grabberConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    public double getVelocityOfGrabber() {
        return m_grabberEncoder.getPosition();
    }

    public double getVelocityOfGrabberFollower() {
        return m_followerEncoder.getPosition();
    }

    public boolean grabberVelocityagrees() {
        return Math.abs(getVelocityOfGrabber() - getVelocityOfGrabberFollower()) < 5;
    }

    public double getClawVoltage() {
        return clawVoltage;
    }

    public void runIntake() {
        if (hasGamePiece()) {
            stopIntake();
        } else {

            clawVoltage = ClawConstants.INTAKE_MOTOR_VOLTAGE;
        }
    }

    public void runOuttake() {
        clawVoltage = -ClawConstants.INTAKE_MOTOR_VOLTAGE;
    }

    public void stopIntake() {
        clawVoltage = 0;
    }

    public boolean hasGamePiece() {
        return isBroken;
    }

    @Override
    public void periodic() {
        m_grabber.set(clawVoltage);
        isBroken = !m_intakeBeamBreak.get();
    }

    // instance or static class???
    private static class Wrist {
        private SparkMax m_wrist = new SparkMax(ClawConstants.WRIST, MotorType.kBrushless);
        private SparkMaxConfig m_wristConfig = new SparkMaxConfig();
        private AbsoluteEncoder m_wristEncoder;

        protected Wrist() {
            m_wristConfig.smartCurrentLimit(Constants.ClawConstants.WRISTCURRENTLIMIT);
            m_wristConfig.idleMode(IdleMode.kBrake);
            m_wristConfig.softLimit.forwardSoftLimitEnabled(false);
            m_wristConfig.softLimit.reverseSoftLimitEnabled(true);
            m_wrist.configure(m_wristConfig, SparkBase.ResetMode.kResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters);
        }

    }
}
