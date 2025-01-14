// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PublicKey;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {
  private NetworkTable limelight;
  private double[] pose = new double[11];
  private double[] targetPose = new double[6];
  private double[] pastPose = new double[11];
  private double[] pastTargetPose = new double[6];

  /** Creates a new ExampleSubsystem. */
  public Limelight() {
  }

  // get valid target
  public boolean getTv() {
    return false;
  }

  // get Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
  // degrees / LL
  public double getTx() {
    return 0.0;
  }

  // get Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
  // degrees / LL2: -24.85 to 24.85 degrees)
  public double getTy() {
    return 0.0;
  }

  // get ID of the primary in-view AprilTag
  public int getID() {
    return 0;
  }

  public void setPriorityID(int id) {

  }

  public double[] getBotPose() {
    return pose;
  }

  public double getDistanceFromPrimaryTarget() {
    return 0.0;
  }

  public void setPriorityID(){
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
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
