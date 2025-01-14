// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
    private SparkMax m_grabber = new SparkMax(ClawConstants.LEFT_GRABBER, MotorType.kBrushless);
    private SparkMax m_follower = new SparkMax(ClawConstants.RIGHT_GRABBER, MotorType.kBrushless);

    private SparkMaxConfig m_grabberConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followerConfig = new SparkMaxConfig();

    private RelativeEncoder m_grabberEncoder;
    private RelativeEncoder m_followerEncoder;

    private double clawVoltage;
    private boolean isBroken;
    private DigitalInput m_intakeBeamBreak = new DigitalInput(ClawConstants.INTAKE_MOTOR_BEAMBREAK);
    private Wrist wrist;
    // In degrees

    /** Creates a new ExampleSubsystem. */
    public Claw() {

    }

    public double getVelocityOfGrabber() {
        return 0;
    }

    public double getVelocityOfGrabberFollower() {
        return 0;
    }

    public boolean grabberVelocityagrees() {
        return false;
    }

    public double getClawVoltage() {
        return clawVoltage;
    }

    public void runIntake() {

    }

    public void runOuttake() {
    }

    public void stopIntake() {
    }

    public boolean hasGamePiece() {
        return isBroken;
    }

    public Wrist getWrist() {
        return wrist;
    }

    public void setWristState(int position) {
    }

    @Override
    public void periodic() {
        m_grabber.set(clawVoltage);
        isBroken = !m_intakeBeamBreak.get();
        if (wrist.isCurrentSpike()) {
            wrist.stopWrist();
        }
    }

    }

    private class Wrist {
        private SparkMax m_wrist = new SparkMax(ClawConstants.WRIST, MotorType.kBrushless);
        private SparkMaxConfig m_wristConfig = new SparkMaxConfig();
        // private AbsoluteEncoder m_wristEncoder = m_wrist.getAbsoluteEncoder();
        private SparkClosedLoopController pidController = m_wrist.getClosedLoopController();
        private double wristVoltage;

        private Wrist() {

            m_wristConfig.smartCurrentLimit(ClawConstants.WRISTCURRENTLIMIT);
            m_wristConfig.idleMode(IdleMode.kBrake);

            m_wristConfig.softLimit.forwardSoftLimit(ClawConstants.MAXIMUMANGLE);
            m_wristConfig.softLimit.reverseSoftLimit(ClawConstants.MINIMUMANGLE);
            m_wristConfig.softLimit.forwardSoftLimitEnabled(true);
            m_wristConfig.softLimit.reverseSoftLimitEnabled(true);
            m_wrist.configure(m_wristConfig, SparkBase.ResetMode.kResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters);
            // AbsoluteEncoderConfig m_wristEncoderConfig = m_wristConfig.absoluteEncoder;
            // m_wristEncoderConfig.zeroOffset(0.0);
            // m_wristEncoderConfig.positionConversionFactor(360.0);
            ClosedLoopConfig pidConfig = m_wristConfig.closedLoop;
            pidConfig.pidf(ClawConstants.KP, ClawConstants.KI, ClawConstants.KD,
                    ClawConstants.KFF);
            pidConfig.outputRange(0, 1.0);
            pidConfig.feedbackSensor(FeedbackSensor.kAnalogSensor);
            pidConfig.maxMotion.maxVelocity(ClawConstants.MAXVELOCITY);
            pidConfig.maxMotion.maxAcceleration(ClawConstants.MAXACCELERATION);
            pidConfig.maxMotion.allowedClosedLoopError(ClawConstants.ALLOWEDERROR);

        }

        public void moveToPosition(double angle) {
            if (angle == 90) {
                wristVoltage = ClawConstants.WRISTVOLTAGE;
            } else if (angle == 0) {
                wristVoltage = -ClawConstants.WRISTVOLTAGE;
            } else {
                throw new IllegalStateException(
                        "Argument in angle must be passed in as the Minimimum angle or Maximum angle (0 or 90)");
            }
            pidController.setReference(wristVoltage, ControlType.kVoltage);
        }

        public void stopWrist() {
            m_wrist.stopMotor();
            wristVoltage = 0;
        }

        public boolean isCurrentSpike() {
            return false;
        }
    }
}