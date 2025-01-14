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
    public static final int MOTOR1 = 1;
    public static final int MOTOR2 = 2;
    public static final int LOW_BEAMBREAK = 2;
    public static final double LOW_POSITION = 0.0;
    public static final double HIGH_POSITION = 1.0;
    public static final double ALGAE_INTAKE = 0.0;
    public static final double CORAL_INTAKE = 0.0;
    public static final double L1 = 0.0;
    public static final double L2 = 1.0;
    public static final double L3 = 2.0;
    public static final double L4 = 3.0;
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

  }

  public static class ClimbConstants {

  }

  public static class LimelightConstants {

  }

  public static class LEDConstants {

  }

}
