// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ElevatorConstants {
    public static final int MOTOR = 1;
  }

  public static class ArmConstants {
    // change positions
    public static final double DEFAULT_ARM_POSITION = 0.0;
    public static final double GROUND_ALGAE_ARM_POSITION = 0.0;
    public static final double CORAL_STATION_ARM_POSITION = 0.0;
    public static final double L1_ARM_POSITION = 0.0;
    public static final double L2_ARM_POSITION = 0.0;
    public static final double L3_ARM_POSITION = 0.0;
    public static final double L4_ARM_POSITION = 0.0;

    public static final double HITING_LEFT_ELEVATOR_ARM_POSITION = 0.0;
    public static final double HITING_RIGHT_ELEVATOR_ARM_POSITION = 0.0;
    public static final double HITING_LEFT_BASE_ARM_POSITION = 0.0;
    public static final double HITING_RIGHT_BASE_ARM_POSITION = 0.0;
  }

  public static class ClawConstants {
    public static final int LEFTGRABBER = 21;
    public static final int RIGHTGRABBER = 22;
    public static final int LEFTINNNER = 23;
    public static final int RIGHTINNER = 24;
    public static final int WRIST = 24;
    public static final int INTAKEMOTORBEAMBREAK = 1;
    // Dummy Values
    public static final int GRABBERCURRENTLIMIT = 5; 
    public static final int WRISTCURRENTLIMIT = 5;
    public static final double INTAKEMOTORVOLTAGE = 0.4;
    public static final double MAXIMUMANGLE = 90.0;
    public static final double MINIMUMANGLE = 0;
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KFF = 0;
    public static final double MAXACCELERATION = 150 ;
    public static final double MAXVELOCITY = 250;
    public static final double ALLOWEDERROR = 1;
  }

  public static class ClimbConstants {

  }

  public static class LimelightConstants {

  }

  public static class LEDConstants {

  }

}
