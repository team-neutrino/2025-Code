
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AnalogSensorConfig;
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
    private static final double[] WRIST_POSITIONS = { 0.0, 90.0 };

    /** Creates a new ExampleSubsystem. */
    public Claw() {
        wrist = new Wrist();
        m_grabberEncoder = m_grabber.getEncoder();
        m_followerEncoder = m_follower.getEncoder();

        // space for current limits
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

    public double getVelocityOfGrabber() {
        return m_grabberEncoder.getVelocity();
    }

    public double getVelocityOfGrabberFollower() {
        return m_followerEncoder.getVelocity();
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
        if (wrist.isCurrentSpike()) {
            wrist.stopWrist();
        }
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
        private SparkClosedLoopController pidController = m_wrist.getClosedLoopController();
        private double wristVoltage;
        SparkAnalogSensor voltageAnalogSensor = m_wrist.getAnalog();

        private Wrist() {
            AnalogSensorConfig voltageAnalogConfig = m_wristConfig.analogSensor;
            m_wristConfig.smartCurrentLimit(ClawConstants.WRIST_CURRENT_LIMIT);
            m_wristConfig.idleMode(IdleMode.kBrake);
            m_wristConfig.softLimit.forwardSoftLimit(ClawConstants.MAXIMUM_ANGLE);
            m_wristConfig.softLimit.reverseSoftLimit(ClawConstants.MINIMUM_ANGLE);
            m_wristConfig.softLimit.forwardSoftLimitEnabled(true);
            m_wristConfig.softLimit.reverseSoftLimitEnabled(true);
            m_wrist.configure(m_wristConfig, SparkBase.ResetMode.kResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters);
            voltageAnalogConfig.velocityConversionFactor(1);
            ClosedLoopConfig pidConfig = m_wristConfig.closedLoop;
            pidConfig.feedbackSensor(FeedbackSensor.kAnalogSensor);
            pidConfig.pidf(ClawConstants.KP, ClawConstants.KI, ClawConstants.KD,
                    ClawConstants.KFF);
            pidConfig.outputRange(0, 1.0);
            pidConfig.feedbackSensor(FeedbackSensor.kAnalogSensor);
            pidConfig.maxMotion.maxVelocity(ClawConstants.MAX_VELOCITY);
            pidConfig.maxMotion.maxAcceleration(ClawConstants.MAX_ACCELERATION);
            pidConfig.maxMotion.allowedClosedLoopError(ClawConstants.ALLOWED_ERROR);
        }

        public void moveToPosition(double angle) {
            if (angle == 90) {
                wristVoltage = ClawConstants.WRIST_VOLTAGE;
            } else if (angle == 0) {
                wristVoltage = -ClawConstants.WRIST_VOLTAGE;
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

        // Test method
        public boolean voltageAgrees() {
            // potentailly use 2nd one
            return Math.abs(wristVoltage - voltageAnalogSensor.getVoltage()) < 0.1;
            // return Math.abs(wristVoltage - m_wrist.getAppliedOutput()) < 0.1;
        }

        public boolean isCurrentSpike() {
            return m_wrist.getOutputCurrent() > ClawConstants.CURRENT_SPIKE_LIMIT;
        }

    }

}