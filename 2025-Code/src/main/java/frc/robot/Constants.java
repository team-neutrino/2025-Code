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
    public static final int kButtonsControllerPort = 1;
  }

  public static class SwerveConstants {
    public static final double MAX_SPEED = 5.7;
    public static final double MAX_ROTATION_SPEED = 1.5 * Math.PI;
  }

  public static class ElevatorConstants {
    public static final int MOTOR1_ID = 2;
    public static final int MOTOR2_ID = 3;
    public static final double LOW_POSITION = 0.0;
    public static final double HIGH_POSITION = 55.0;
    public static final double ALGAE_INTAKE = 0.0;
    public static final double CORAL_INTAKE = 0.0;
    public static final double REMOVE_ALGAE_L2 = 0.0;
    public static final double REMOVE_ALGAE_L3 = 0.0;
    public static final double L1 = 6.0; // placeholder
    public static final double L2 = 18.0; // placeholder
    public static final double L3 = 30.0; // placeholder
    public static final double L4 = 54.0; // placeholder
    public static final double ARM_WILL_NOT_HIT_BASE_HEIGHT = 1.0; // placeholder
    public static final double STAGE_1_LENGTH = 29; // inches
    public static final double STAGE_2_LENGTH = 26; // inches
    public static final double GEAR_RATIO = 50 / 7; // 7.41:1
    public static final double FLOOR_TO_ELEVATOR_TOP = 70.88;
    public static final double FLOOR_TO_TOP_OF_BOTTOM_TUBE = 11.88;
    public static final double L1_TO_L2 = 13.84;
    public static final double L2_TO_L3 = 15.87;
    public static final double L3_TO_L4 = 24.28;
    public static final double STAGE_1_MASS = 3.5;
    public static final double STAGE_2_MASS = 1;
    public static final double ARM_AND_SCORING_MASS = 2;
  }

  public static class ArmConstants {
    public static final int MOTOR_ID = 10;
    public static final double DEFAULT_ARM_POSITION = 0.0;
    public static final double GROUND_ALGAE_ARM_POSITION = 0.0;
    public static final double CORAL_STATION_ARM_POSITION = 0.0;
    public static final double L1_ARM_POSITION = 0.0;
    public static final double L2_ARM_POSITION = 0.0;
    public static final double L3_ARM_POSITION = 0.0;
    public static final double L4_ARM_POSITION = 0.0;

    public static final double HITTING_LEFT_BOTTOM_ELEVATOR_ARM_POSITION = 0.0;
    public static final double HITTING_RIGHT_BOTTOM_ELEVATOR_ARM_POSITION = 0.0;
    public static final double HITTING_LEFT_TOP_ELEVATOR_ARM_POSITION = 0.0;
    public static final double HITTING_RIGHT_TOP_ELEVATOR_ARM_POSITION = 0.0;
    public static final double HITTING_LEFT_BASE_ARM_POSITION = 0.0;
    public static final double HITTING_RIGHT_BASE_ARM_POSITION = 0.0;

    public static final double ARM_ENCODER_ZERO_OFFSET = 0;
    public static final int ARM_CURRENT_LIMIT = 20;

    public static final double Arm_kp = 1;
    public static final double Arm_ki = 0;
    public static final double Arm_kd = 0;
    public static final double ArmIZone = 0;
    public static final int REEF_L2_DESCORE_POSITION = 0;
    public static final int REEF_L3_DESCORE_POSITION = 0;
  }

  public static class ClawConstants {
    public static final int LEFT_GRABBER = 21;
    public static final int RIGHT_GRABBER = 22;
    public static final int INTAKE_MOTOR_BEAMBREAK = 0;
    public static final int GRABBER_CURRENT_LIMIT = 20;
    public static final double INTAKE_MOTOR_VOLTAGE = 0.2;

  }

  public static class WristConstants {
    public static final int MOTOR_ID = 25;
    public static final int CURRENT_LIMIT = 15;
    public static final double HARDSTOP_CURRENT_LIMIT = .3;
    public static final double VOLTAGE = 0.3;
    public static final int INTAKE_POS = 90;
    public static final int SCORING_POS = 0;
    public static final double RAMP_RATE = 0.25;
  }

  public static class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 30;
    public static final int CLIMB_MOTOR_ID2 = 31;
    public static final int CLIMB_MOTOR_ID3 = 32;
    public static final int CLIMB_MOTOR_ID4 = 34;

    public static final int CLIMB_CURRENT_LIMIT = 0;
    // int not correct
    public static final int HOLD_CURRENT_LIMIT = 0;
    // int not correct

    public static final int LOCK_LIMIT_SWITCH = 1;
  }

  public static class LimelightConstants {
    // placeholder values (in meters)
    public static final double CAMERA_FORWARD_OFFSET = 0.0;
    public static final double CAMERA_SIDE_OFFSET = 0.0;
    public static final double CAMERA_HEIGHT_OFFSET = 0.0;
    // placeholdere values (in degrees)
    public static final double CAMERA_ROLL_OFFSET = 0.0;
    public static final double CAMERA_PITCH_OFFSET = 0.0;
    public static final double CAMERA_YAW_OFFSET = 0.0;
  }

  public static class LEDConstants {

  }

  public static class AprilTagConstants {
    public final class RED_ALLIANCE_IDS {

    }

    public final class BLUE_ALLIANCE_IDS {

    }

  }

}