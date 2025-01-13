// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private DigitalInput m_intakeBeamBreak = new DigitalInput(ClawConstants.INTAKEMOTORBEAMBREAK);
    private Wrist wrist;
    // In degrees
    private static final double[] WRIST_POSITIONS = { 0.0, 45.0, 90.0 };

    /** Creates a new ExampleSubsystem. */
    public Claw() {
        wrist = new Wrist();
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
        m_follower.configure(m_followerConfig, SparkBase.ResetMode.kResetSafeParameters,
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
            clawVoltage = ClawConstants.INTAKEMOTORVOLTAGE;
        }
    }

    public void runOuttake() {
        clawVoltage = -ClawConstants.INTAKEMOTORVOLTAGE;
    }

    public void stopIntake() {
        clawVoltage = 0;
    }

    public boolean hasGamePiece() {
        return isBroken;
    }

    public Wrist getWrist() {
        return wrist;
    }

    public void setWristState(int position) {
        wrist.moveToPosition(WRIST_POSITIONS[position]);
    }

    @Override
    public void periodic() {
        m_grabber.set(clawVoltage);
        isBroken = !m_intakeBeamBreak.get();
    }

    public Command defaultCommandGrabber() {
        return new RunCommand(() -> stopIntake(), this);
    }

    public Command defaultCommandWrist() {
        return new RunCommand(() -> wrist.moveToPosition(WRIST_POSITIONS[0]), this);
    }

    public Command rotateWristto90() {
        return new RunCommand(() -> wrist.moveToPosition(WRIST_POSITIONS[2]), this);
    }

    public Command intakeGamePiece() {
        return new RunCommand(() -> runIntake(), this);
    }

    private class Wrist {
        private SparkMax m_wrist = new SparkMax(ClawConstants.WRIST, MotorType.kBrushless);
        private SparkMaxConfig m_wristConfig = new SparkMaxConfig();
        private AbsoluteEncoder m_wristEncoder = m_wrist.getAbsoluteEncoder();
        private SparkClosedLoopController pidController = m_wrist.getClosedLoopController();

        private Wrist() {

            m_wristConfig.smartCurrentLimit(ClawConstants.WRISTCURRENTLIMIT);
            m_wristConfig.idleMode(IdleMode.kBrake);

            m_wristConfig.softLimit.forwardSoftLimit(ClawConstants.MAXIMUMANGLE);
            m_wristConfig.softLimit.reverseSoftLimit(ClawConstants.MINIMUMANGLE);
            m_wristConfig.softLimit.forwardSoftLimitEnabled(true);
            m_wristConfig.softLimit.reverseSoftLimitEnabled(true);
            m_wrist.configure(m_wristConfig, SparkBase.ResetMode.kResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters);
            AbsoluteEncoderConfig m_wristEncoderConfig = m_wristConfig.absoluteEncoder;
            m_wristEncoderConfig.zeroOffset(0.0);
            m_wristEncoderConfig.positionConversionFactor(360.0);
            ClosedLoopConfig pidConfig = m_wristConfig.closedLoop;
            pidConfig.pidf(ClawConstants.KP, ClawConstants.KI, ClawConstants.KD, ClawConstants.KFF);
            pidConfig.outputRange(-1.0, 1.0);
            pidConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            pidConfig.maxMotion.maxVelocity(ClawConstants.MAXVELOCITY);
            pidConfig.maxMotion.maxAcceleration(ClawConstants.MAXACCELERATION);
            pidConfig.maxMotion.allowedClosedLoopError(ClawConstants.ALLOWEDERROR);
        }

        public void moveToPosition(double angle) {
            if (angle < ClawConstants.MINIMUMANGLE) {
                angle = ClawConstants.MINIMUMANGLE;
            } else if (angle > ClawConstants.MAXIMUMANGLE) {
                angle = ClawConstants.MAXIMUMANGLE;
            }
            pidController.setReference(angle, ControlType.kPosition);
        }

        public void stopWrist() {
            m_wrist.stopMotor();
        }

        public double getCurrentPosition() {
            return m_wristEncoder.getPosition();
        }

        public boolean verifyPosition(double angle) {
            return Math.abs(getCurrentPosition() - angle) <= ClawConstants.ALLOWEDERROR;
        }

    }

}
