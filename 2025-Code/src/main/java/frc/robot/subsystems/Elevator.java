// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private SparkFlex m_motor1 = new SparkFlex(ElevatorConstants.MOTOR1_ID, MotorType.kBrushless);
  private SparkFlex m_motor2 = new SparkFlex(ElevatorConstants.MOTOR2_ID,
      MotorType.kBrushless);
  private SparkRelativeEncoder m_encoder;
  private SparkLimitSwitch m_lowLimit;
  private SparkClosedLoopController m_pid = m_motor1.getClosedLoopController();
  private SparkFlexConfig m_config = new SparkFlexConfig();
  private SparkFlexConfig m_followerConfig = new SparkFlexConfig();

  private double m_target = 0.0;

  // MXP i2c board. Don't use the RIO onboard because it has lockup issues
  private final I2C.Port i2cPort = I2C.Port.kMXP;
  // https://codedocs.revrobotics.com/java/com/revrobotics/colorsensorv3
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  public Elevator() {
    m_config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    m_config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.0, 0.0, 0.0);
    m_motor1.configure(m_config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_followerConfig.follow(ElevatorConstants.MOTOR1_ID);
    m_followerConfig.apply(m_config);
    m_motor2.configure(m_followerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private void setTargetHeight(double target) {
    m_target = target;
  }

  private void adjustElevator(double target) {
    m_pid.setReference(target, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardCalculation());
  }

  private double feedForwardCalculation() {
    return 0.0;
  }

  private void resetEncoder(double position) {
    m_encoder.setPosition(position);
  }

  private boolean isLowPosition() {
    return false;
  }

  public boolean isHighPosition() {
    return false;
  }

  public Command moveElevatorCommand(double height) {
    return run(
        () -> {
          setTargetHeight(height);
        });
  }

  @Override
  public void periodic() {
    adjustElevator(m_target);
    
    // color sensor
    Color detectedColor = m_colorSensor.getColor();
    double rawValue = m_colorSensor.getIR();

    // 0-2047
    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", rawValue);
    SmartDashboard.putNumber("Proximity", proximity);
  }

  @Override
  public void simulationPeriodic() {

  }
}
