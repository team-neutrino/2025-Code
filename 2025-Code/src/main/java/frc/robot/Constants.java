// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.DriveToPointCalculator;

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
  public static class GlobalConstants {
    public static Optional<Boolean> redAlliance = Optional.empty();
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kButtonsControllerPort = 1;
    public static final int kPitControllerPort = 3;
  }

  public final class CANRateConstants {
    public static final int FASTEST_5MS = 5;
    public static final int FAST_10MS = 10;
  }

  public static class SwerveConstants {
    public static final double MAX_SPEED = 5.7;
    public static final double SLOW_SWERVE_SPEED = 1;
    public static final double MAX_ROTATION_SPEED = 1.5 * Math.PI;
    public static final double SLOW_ROTATION_SPEED = 0.8 * Math.PI;
    public static final double DRIVE_ASSIST_KP = 4;
    public static final double[] HEXAGON_ANGLES = { Integer.MAX_VALUE, -1, -1, -1, -1, -1, 120, 180, -120, -60, 0, 60,
        -1, -1, -1, -1, -1, -120, 180, 120, 60, 0, -60 };
    public static final double APRILTAG_ALIGN_LIMIT = 1.5;
    public static final double AUTO_ALIGN_P = 4;
    public static final double AUTO_ALIGN_D = .02;
    public static final double STALL_TORQUE = 7.09;
    public static final double STALL_CURRENT = 366;
    public static final double FREE_CURRENT_AMPS = 2;
    public static final double FREE_SPEED_RADS = (6000 / 60) * Math.PI * 2;
    public static final int NUM_MOTORS_GEARBOX = 1;
    public static final double NOMINAL_VOLTAGE = 12;
    public static final double WHEEL_RADIUS = 2;
    public static final double DRIVE_GEAR_RATIO = 5.6;
    public static final double REEF_OFFSET = Units.inchesToMeters(12.94) / 2;
    public static final double isAlignedError = 0;
    public static final double DRIVE_TO_POINT_P = 3;
    public static final double AT_POINT_TOLERANCE = 0.05;
  }

  public static class ElevatorConstants {
    public static final int MOTOR_ID = 2;
    public static final int FOLLOWER_ID = 3;
    public static final int CURRENT_LIMIT = 60;
    public static final double DEFAULT = 0.0;
    public static final double BOTTOM_POSITION = 0.0;
    public static final double STAGE_ONE_UP = 26.0;
    public static final double CORAL_INTAKE = 34;
    public static final double REMOVE_ALGAE = 0.0;
    public static final double L1 = 14.0;
    public static final double L2 = 26.0;
    public static final double L3 = 36.0;
    public static final double L4 = 54.9;
    public static final double ARM_WILL_NOT_HIT_BASE_HEIGHT = 20.0;
    public static final double STAGE_1_LENGTH = 29;
    public static final double STAGE_2_LENGTH = 26;
    public static final double GEAR_RATIO = 50 / 7; // 7.41:1
    public static final double FLOOR_TO_ELEVATOR_TOP = 70.88;
    public static final double FLOOR_TO_TOP_OF_BOTTOM_TUBE = 11.88;
    public static final double MAX_VELOCITY = 4000;
    public static final double MAX_ACCELERATION = 2000;
    public static final double ALLOWED_ERROR = 0.1;
    public static final double P_VAL = 0.11;
    public static final double I_VAL = 0.0;
    public static final double D_VAL = 0.0;
    public static final double STAGE_1_FF = 0.25;
    public static final double STAGE_2_FF = 0.30;
    public static final double HEIGHT_TOLERANCE = 0.6;
  }

  public static class ArmConstants {
    public static final double STARTING_POSITION = 180;
    public static final double DEFAULT_POSITION = 160;
    public static final double DEFAULT_BACK_POSITION = 250;
    public static final double GROUND_ALGAE_POSITION = 95;
    public static final double CORAL_STATION_POSITION = 328;
    public static final double L1_UNDERHAND = 335.0;
    public static final double L1_POSITION = 140.0;
    public static final double L2_POSITION = 125.0;
    public static final double L3_POSITION = 135.0;
    public static final double L4_POSITION = 150.0;
    public static final double EVACUATE_ANGLE = 15.0;
    public static final double REEF_L2_DESCORE_POSITION = 0.0;
    public static final double REEF_L3_DESCORE_POSITION = 0.0;
    public static final double DRIVING_ANGLE_TOLERANCE = 2.0;

    public static final double HITTING_LOW_HARD_LIMIT = 90;
    public static final double HITTING_HIGH_HARD_LIMIT = 270;
    public static final double ALMOST_BACK_LIMIT = 200;
    public static final double ALMOST_FRONT_LIMIT = 170;

    public static final int MOTOR_ID = 10;

    public static final double ENCODER_ZERO_OFFSET = 0;
    public static final int CURRENT_LIMIT = 60;

    public static final double MAX_VELOCITY = 40000;
    public static final double MAX_ACCELERATION = 50000;
    public static final double ALLOWED_ERROR = 0.7;

    public static final double GAIN_THRESHOLD = 6;

    public static final double kp = 0.03;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double kp1 = 0.05;
    public static final double ki1 = 0;
    public static final double kd1 = 0;
    public static final double ArmIZone = 2;

    public static final double FFCONSTANT = 0.04;

    public static final double GEAR_RATIO = 125;
  }

  public static class CoralConstants {
    public static final int LEFT_GRABBER = 21;
    public static final int GRABBER_CURRENT_LIMIT = 20;
    public static final double INTAKE_MOTOR_VOLTAGE = 0.7;
    public static final double HOLD_PIECE_VOLTAGE = .1;
    public static final double SLOW_MOTOR_VOLTAGE = 0.3;

    public static final int COLOR_SENSOR = 27;

    public static final double PROXIMITY = 0.05;
  }

  public static class ClimbConstants {
    public static final int MAIN_MOTOR_ID = 31;
    public static final int FOLLOW_MOTOR_ID = 30;
    public static final int LOCK_MOTOR_ID = 32;
    public static final int RATCHET_PORT = 0;

    public static final int CLIMB_CURRENT_LIMIT = 80;
    public static final int LOCK_CURRENT_LIMIT = 20;

    public static final int CLIMB_UP_POSITION = 35;
    public static final int CLIMB_DOWN_POSITION = 0;
    public static final int RESET_CLIMB_ROTATION = -105;

    public static final int CLIMB_POSITION_TOLERANCE = 2;
    public static final int LOCK_MOTOR_POSITION_SAFETY = -10;

    public static final int LOCK_POSITION = 15;
    public static final int UNLOCK_POSITION = -15;
    public static final int RESET_LOCK_POSITION = 0;

    public static final double RATCHET_LOCK_POSITION = 0.7;
    public static final double RATCHET_UNLOCK_POSITION = 1;

    public static final double CLIMB_kP = 0.75;
    public static final double CLIMB_kI = 0;
    public static final double CLIMB_kD = 0;

    public static final double LOCK_kP = 0.75;
    public static final double LOCK_kI = 0;
    public static final double LOCK_kD = 0;

    public static final double VELOCITY = 0;
    public static final double ACCELERATION = 0;
    public static final double JERK = 0;
    // subject to change

    public static final double COMMAND_WAIT_TIME = 0.5;
  }

  public static class LimelightConstants {
    // placeholder values (in meters)
    public static final double CAMERA_FORWARD_OFFSET = 0.216;
    public static final double CAMERA_SIDE_OFFSET = 0.121;
    public static final double CAMERA_HEIGHT_OFFSET = 0.342;
    // placeholder values (in degrees)
    public static final double CAMERA_ROLL_OFFSET = 0.0;
    public static final double CAMERA_PITCH_OFFSET = 0.0;
    public static final double CAMERA_YAW_OFFSET = 0.0;
    // placeholder values (in meters)
    public static final double CAMERA2_FORWARD_OFFSET = -0.216;
    public static final double CAMERA2_SIDE_OFFSET = 0.121;
    public static final double CAMERA2_HEIGHT_OFFSET = 0.7874;
    // placeholder values (in degrees)
    public static final double CAMERA2_ROLL_OFFSET = 0.0;
    public static final double CAMERA2_PITCH_OFFSET = 30.0;
    public static final double CAMERA2_YAW_OFFSET = 180.0;

    public static final String LL_REEF = "limelight-limeade";
    public static final String LL_STATION = "limelight-limebee";
  }

  public static class LEDConstants {
    public enum States {
      DEFAULT,
      LOCKCLIMB,
      CLIMBING
    }
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

  public static class DriveToPoint {
    public static final double offsetOfArm = .32;
    public static final double offsetToReef = .56;
    public static final double reefWidth = 0.33;

    // the number correspond to the april tag on the object
    public static final Pose2d RED_PLAYER_STATION_1 = new Pose2d(16.08, 0.99, Rotation2d.fromDegrees(126));
    public static final Pose2d RED_PLAYER_STATION_2 = new Pose2d(16.08, 7.06, Rotation2d.fromDegrees(-126));

    public static final Pose2d BLUE_PLAYER_STATION_12 = new Pose2d(1.47, 0.99, Rotation2d.fromDegrees(54));
    public static final Pose2d BLUE_PLAYER_STATION_13 = new Pose2d(1.47, 7.06, Rotation2d.fromDegrees(-54));

    public static final Pose2d RED_REEF_6A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[6], true);
    public static final Pose2d RED_REEF_6B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[6], false);
    public static final Pose2d RED_REEF_7A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[7], true);
    public static final Pose2d RED_REEF_7B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[7], false);
    public static final Pose2d RED_REEF_8A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[8], true);
    public static final Pose2d RED_REEF_8B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[8], false);
    public static final Pose2d RED_REEF_9A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[9], true);
    public static final Pose2d RED_REEF_9B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[9], false);
    public static final Pose2d RED_REEF_10A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[10], true);
    public static final Pose2d RED_REEF_10B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[10], false);
    public static final Pose2d RED_REEF_11A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[11], true);
    public static final Pose2d RED_REEF_11B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[11], false);

    public static final Pose2d BLUE_REEF_17A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[17], false);
    public static final Pose2d BLUE_REEF_17B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[17], true);
    public static final Pose2d BLUE_REEF_18A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[18], false);
    public static final Pose2d BLUE_REEF_18B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[18], true);
    public static final Pose2d BLUE_REEF_19A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[19], false);
    public static final Pose2d BLUE_REEF_19B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[19], true);
    public static final Pose2d BLUE_REEF_20A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[20], false);
    public static final Pose2d BLUE_REEF_20B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[20], true);
    public static final Pose2d BLUE_REEF_21A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[21], false);
    public static final Pose2d BLUE_REEF_21B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[21], true);
    public static final Pose2d BLUE_REEF_22A = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[22], false);
    public static final Pose2d BLUE_REEF_22B = DriveToPointCalculator
        .CalculatePoint(FieldConstants.APRIL_TAG_POSITIONS[22], true);

    // Don't reorder this list
    public static final List<Pose2d> POSE_LIST = List.of(RED_PLAYER_STATION_1, RED_PLAYER_STATION_2,
        BLUE_PLAYER_STATION_12, BLUE_PLAYER_STATION_13, RED_REEF_6A, RED_REEF_6B, RED_REEF_7A, RED_REEF_7B, RED_REEF_8A,
        RED_REEF_8B, RED_REEF_9A, RED_REEF_9B, RED_REEF_10A, RED_REEF_10B, RED_REEF_11A, RED_REEF_11B, BLUE_REEF_17A,
        BLUE_REEF_17B, BLUE_REEF_18A, BLUE_REEF_18B, BLUE_REEF_19A, BLUE_REEF_19B, BLUE_REEF_20A, BLUE_REEF_20B,
        BLUE_REEF_21A, BLUE_REEF_21B, BLUE_REEF_22A, BLUE_REEF_22B);

    public static final List<Pose2d> RED_REEF = List.of(RED_REEF_6A, RED_REEF_6B, RED_REEF_7A, RED_REEF_7B, RED_REEF_8A,
        RED_REEF_8B, RED_REEF_9A, RED_REEF_9B, RED_REEF_10A, RED_REEF_10B, RED_REEF_11A, RED_REEF_11B);

    public static final List<Pose2d> BLUE_REEF = List.of(BLUE_REEF_22B, BLUE_REEF_22A, BLUE_REEF_21B, BLUE_REEF_21A,
        BLUE_REEF_20B, BLUE_REEF_20A, BLUE_REEF_19B, BLUE_REEF_19A, BLUE_REEF_18B, BLUE_REEF_18A, BLUE_REEF_17B,
        BLUE_REEF_17A);
  }

}