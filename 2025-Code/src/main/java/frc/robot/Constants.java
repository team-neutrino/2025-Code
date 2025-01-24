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
    public static final double STALL_TORQUE = 7.09;
    public static final double STALL_CURRENT = 366;
    public static final double FREE_CURRENT_AMPS = 2;
    public static final double FREE_SPEED_RADS = (6000 / 60) * Math.PI * 2;
    public static final int NUM_MOTORS_GEARBOX = 1;
    public static final double NOMINAL_VOLTAGE = 12;
    public static final double WHEEL_RADIUS = 2;
    public static final double DRIVE_GEAR_RATIO = 5.6;
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
    public static final double MAX_VELOCITY = 4000;
    public static final double MAX_ACCELERATION = 1000;
    public static final double ALLOWED_ERROR = 0.1;
    public static final double P_VAL = 0.1;
    public static final double I_VAL = 0.0;
    public static final double D_VAL = 0.0;
  }

  public static class ArmConstants {
    // change positions
    public static final double DEFAULT_POSITION = 250;
    public static final double GROUND_ALGAE_POSITION = 0.0;
    public static final double CORAL_STATION_POSITION = 0.0;
    public static final double L1_POSITION = 180;
    public static final double L2_POSITION = 0.5;
    public static final double L3_POSITION = 0.0;
    public static final double L4_POSITION = 0.0;
    public static final double REEF_L2_DESCORE_POSITION = 0.0;
    public static final double REEF_L3_DESCORE_POSITION = 0.0;

    public static final double HITTING_LOW_HARD_LIMIT = 75;
    public static final double HITTING_HIGH_HARD_LIMIT = 285;

    public static final int MOTOR_ID = 10;

    public static final double ENCODER_ZERO_OFFSET = 0;
    public static final int CURRENT_LIMIT = 20;

    public static final double MAX_VELOCITY = 10;
    public static final double MAX_ACCELERATION = 20;
    public static final double ALLOWED_ERROR = 0.1;

    public static final double kp = 0.005;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double ArmIZone = 0;

    public static final double FFCONSTANT = 0;

    public static final double GEAR_RATIO = 166.67;
  }

  public static class ClawConstants {
    public static final int LEFT_GRABBER = 21;
    public static final int RIGHT_GRABBER = 22;
    public static final int INTAKE_MOTOR_BEAMBREAK = 0;
    public static final int GRABBER_CURRENT_LIMIT = 20;
    public static final double INTAKE_MOTOR_VOLTAGE = 0.2;

    public static final int COLOR_SENSOR = 27;
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
    public static final int CLIMB_ENCODER_ID = 33;

    public static final int CLIMB_CURRENT_LIMIT = 0;
    public static final int LOCK_CURRENT_LIMIT = 1;
    // subject to change

    public static final double LOCK_SPEED = 0.1;
    // subject to change

    public static final double ARM_UP_TICKS = 1024;
    public static final double ARM_DOWN_TICKS = -1024;

    public static final int CLIMB_RATCHET_PORT = 0;
    public static final int LOCK_RATCHET_PORT = 1;
    // subject to change
  }

  public static class LimelightConstants {
    // placeholder values (in meters)
    public static final double CAMERA_FORWARD_OFFSET = 0.0;
    public static final double CAMERA_SIDE_OFFSET = 0.0;
    public static final double CAMERA_HEIGHT_OFFSET = 0.0;
    // placeholder values (in degrees)
    public static final double CAMERA_ROLL_OFFSET = 0.0;
    public static final double CAMERA_PITCH_OFFSET = 0.0;
    public static final double CAMERA_YAW_OFFSET = 0.0;
    // placeholder values (in meters)
    public static final double CAMERA2_FORWARD_OFFSET = 0.0;
    public static final double CAMERA2_SIDE_OFFSET = 0.0;
    public static final double CAMERA2_HEIGHT_OFFSET = 0.0;
    // placeholdere values (in degrees)
    public static final double CAMERA2_ROLL_OFFSET = 0.0;
    public static final double CAMERA2_PITCH_OFFSET = 0.0;
    public static final double CAMERA2_YAW_OFFSET = 0.0;
  }

  public static class LEDConstants {

  }

  public static class AprilTagConstants {
    public final class RED_ALLIANCE_IDS {
      public static final int SOURCE = 1;
      public static final int SOURCE_PROCESSOR_SIDE = 2;
      public static final int PROCESSSOR = 3;
      public static final int REEF_FACING_SOURCE = 6;
      public static final int REEF_FACING_ALLIANCE = 7;
      public static final int REEF_FACING_SOURCE_PROCESSOR_SIDE = 8;
      public static final int REEF_FACING_PROCESSOR = 9;
      public static final int REEF_FACING_BARGE = 10;
      public static final int REEF_FACING_CAGES = 11;
    }

    public final class BLUE_ALLIANCE_IDS {
      public static final int SOURCE = 13;
      public static final int SOURCE_PROCESSOR_SIDE = 12;
      public static final int PROCESSSOR = 16;
      public static final int REEF_FACING_SOURCE = 19;
      public static final int REEF_FACING_ALLIANCE = 18;
      public static final int REEF_FACING_SOURCE_PROCESSOR_SIDE = 17;
      public static final int REEF_FACING_PROCESSOR = 22;
      public static final int REEF_FACING_BARGE = 21;
      public static final int REEF_FACING_CAGES = 20;
    }

  }

}