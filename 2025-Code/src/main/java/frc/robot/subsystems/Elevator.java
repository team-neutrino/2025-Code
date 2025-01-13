// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private SparkFlex m_motor = new SparkFlex(ElevatorConstants.MOTOR, MotorType.kBrushless);
  private SparkRelativeEncoder m_encoder;
  private SparkLimitSwitch m_limit;
  private DigitalInput m_lowBeambreak = new DigitalInput(ElevatorConstants.LOW_BEAMBREAK);

  public Elevator() {

  }

  private void setTargetHeight(double height) {

  }

  private void resetEncoder(double position) {
    m_encoder.setPosition(position);
  }

  private boolean lowPosition() {
    return m_lowBeambreak.get();
  }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
