
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {

    private SparkMax m_grabber = new SparkMax(ClawConstants.LEFT_GRABBER, MotorType.kBrushless);
    private SparkMax m_follower = new SparkMax(ClawConstants.RIGHT_GRABBER, MotorType.kBrushless);

    private SparkMaxConfig m_grabberConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followerConfig = new SparkMaxConfig();

    private RelativeEncoder m_grabberEncoder;
    private RelativeEncoder m_followerEncoder;

    private double intakeVoltage;
    private boolean isBroken;
    private DigitalInput m_intakeBeamBreak = new DigitalInput(ClawConstants.INTAKE_MOTOR_BEAMBREAK);
    private Wrist wrist;
    private static final double[] WRIST_POSITIONS = { 0.0, 90.0 };

    public Claw() {
        wrist = new Wrist();
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

    public double getVelocityOfGrabber() {
        return m_grabberEncoder.getVelocity();
    }

    public double getVelocityOfGrabberFollower() {
        return m_followerEncoder.getVelocity();
    }

    public boolean grabberVelocityagrees() {
        return Math.abs(getVelocityOfGrabber() - getVelocityOfGrabberFollower()) < 5;
    }

    public double getIntakeVoltage() {
        return intakeVoltage;
    }

    public void runIntake() {
        if (hasGamePiece()) {
            stopIntake();
        } else {
            intakeVoltage = ClawConstants.INTAKE_MOTOR_VOLTAGE;
        }
    }

    public void runOuttake() {
        intakeVoltage = -ClawConstants.INTAKE_MOTOR_VOLTAGE;
    }

    public void stopIntake() {
        intakeVoltage = 0;
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
        m_grabber.set(intakeVoltage);
        isBroken = !m_intakeBeamBreak.get();
    }

    public Command clawAndWristDefaultCommand() {
        return new RunCommand(() -> {
            stopIntake();
            wrist.stopWrist();
        }, this);
    }

    public Command rotateWristTo0() {
        return new RunCommand(() -> wrist.moveToPosition(WRIST_POSITIONS[0]), this);
    }

    public Command intakeGamePiece() {
        return run(() -> runIntake());
    }

    public Command outakeGamePiece() {
        return run(() -> runOuttake());
    }

    private class Wrist {
        private SparkMax m_wrist = new SparkMax(ClawConstants.WRIST, MotorType.kBrushless);
        private SparkMaxConfig m_wristConfig = new SparkMaxConfig();
        private double wristVoltage;
        private double m_angle;
        private boolean hasCurrentSpiked;

        private Wrist() {
            m_wristConfig.smartCurrentLimit(ClawConstants.WRIST_CURRENT_LIMIT);
            m_wristConfig.idleMode(IdleMode.kBrake);
            m_wristConfig.openLoopRampRate(0.25);
            m_wrist.configure(m_wristConfig, SparkBase.ResetMode.kResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters);
        }

        public void moveToPosition(double angle) {
            if (angle != m_angle) {
                wrist.hasCurrentSpiked = false;
            }
            m_angle = angle;
            if (angle == 90) {
                wristVoltage = ClawConstants.WRIST_VOLTAGE;
            } else if (angle == 0) {
                wristVoltage = -ClawConstants.WRIST_VOLTAGE;
            } else {
                throw new IllegalStateException(
                        "Argument in angle must be passed in as the Minimimum angle or Maximum angle (0 or 90)");
            }
            if (wrist.hasCurrentSpiked()) {
                wrist.stopWrist();
            } else {
                m_wrist.set(wristVoltage);
                wrist.hasCurrentSpiked = wrist.isCurrentSpike();
            }
        }

        public void stopWrist() {
            m_wrist.stopMotor();
            wristVoltage = 0;
            m_wrist.set(wristVoltage);
        }

        public boolean voltageAgrees() {
            return Math.abs(wristVoltage - m_wrist.getOutputCurrent()) < 0.1;
        }

        public boolean isCurrentSpike() {
            return m_wrist.getOutputCurrent() > ClawConstants.WRIST_CURRENT_LIMIT;
        }

        public boolean hasCurrentSpiked() {
            return hasCurrentSpiked;
        }

    }

}