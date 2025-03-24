// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Enumeration;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import static frc.robot.util.DriveToPointCalculator.*;

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
                public static final double SLOW_SWERVE_SPEED = 3;
                public static final double MAX_ROTATION_SPEED = 1.5 * Math.PI;
                public static final double SLOW_ROTATION_SPEED = 0.8 * Math.PI;
                public static final double DRIVE_ASSIST_KP = 4;
                public static final double[] HEXAGON_ANGLES = { Integer.MAX_VALUE, -1, -1, -1, -1, -1, 120, 180, -120,
                                -60, 0,
                                60,
                                -1, -1, -1, -1, -1, -120, 180, 120, 60, 0, -60 };
                public static final double APRILTAG_ALIGN_LIMIT = 1.5;
                public static final double AUTO_ALIGN_P = 4;
                public static final double AUTO_ALIGN_D = 0.02;
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
                public static final double DRIVE_TO_POINT_INTAKE_P = 5;
                public static final double AT_POINT_TOLERANCE = 0.05;
        }

        public static class ElevatorConstants {
                public static final int MOTOR_ID = 2;
                public static final int FOLLOWER_ID = 3;
                public static final int CURRENT_LIMIT = 60;
                public static final double DEFAULT_NO_CORAL = 17.0;
                public static final double DEFAULT_WITH_CORAL = 0;
                public static final double BOTTOM_POSITION = 0.0;
                public static final double STAGE_ONE_UP = 26.0;
                public static final double CORAL_INTAKE = 34;
                public static final double REMOVE_ALGAE_L2 = 30.0;
                public static final double REMOVE_ALGAE_L3 = 50.0;
                public static final double SCORE_ALGAE_BARGE = 54.9;
                public static final double SCORE_ALGAE_PROCESSOR = 18.0;
                public static final double L1 = 11.0;
                public static final double L2 = 16.0;
                public static final double L3 = 31.0;
                public static final double L4 = 54.9;
                public static final double ARM_WILL_NOT_HIT_BASE_HEIGHT = 20.0;
                public static final double STAGE_1_LENGTH = 29;
                public static final double STAGE_2_LENGTH = 26;
                public static final double GEAR_RATIO = 50 / 7;
                public static final double FLOOR_TO_ELEVATOR_TOP = 70.88;
                public static final double FLOOR_TO_TOP_OF_BOTTOM_TUBE = 11.88;
                public static final double MAX_VELOCITY = 4000;
                public static final double MAX_ACCELERATION = 6000;
                public static final double ALLOWED_ERROR = 0.1;
                public static final double P_VAL = 0.1;
                public static final double I_VAL = 0.0;
                public static final double D_VAL = 1.0;
                public static final double P_VAL_ALGAE = 0.05;
                public static final double I_VAL_ALGAE = 0.0;
                public static final double D_VAL_ALGAE = 0.0;
                public static final double STAGE_1_FF = 0.25;
                public static final double STAGE_2_FF = 0.30;
                public static final double HEIGHT_TOLERANCE = 0.6;
                public static final double SLOW_MOVE_THRESHOLD = 45.0;
                public static final double SAFE_HEIGHT_ALGAE = 30;
                public static final double SAFE_HEIGHT_NO_ALGAE = 16;
        }

        public static class ArmConstants {
                public static final int MOTOR_ID = 10;

                public static final double STARTING_POSITION = 180;
                public static final double DEFAULT_POSITION = 160;
                public static final double DEFAULT_NO_GP = 170;
                public static final double SAFE_BACK_POS = 230;
                public static final double GROUND_ALGAE_POSITION = 95;
                public static final double CORAL_STATION_POSITION = 323;
                public static final double L1_UNDERHAND = 335.0;
                public static final double L1_POSITION = 130.0;
                public static final double L2_POSITION = 141.0;
                public static final double L3_POSITION = 141.0;
                public static final double L4_POSITION = 140.0;
                public static final double BARGE_POSITION = 159.0;
                public static final double PROCESSOR_POSITION = 270.0;
                public static final double EVACUATE_ANGLE = 15.0;
                public static final double REEF_DESCORE_POSITION = 50.0;
                public static final double ALGAE_FRONT_SAFE_ANGLE = 90;

                public static final double DRIVING_ANGLE_TOLERANCE = 2.0;

                public static final double ENCODER_ZERO_OFFSET = 0;
                public static final int CURRENT_LIMIT = 60;

                public static final double MAX_VELOCITY = 40000;
                public static final double MAX_ACCELERATION = 50000;
                public static final double ALLOWED_ERROR = 0.7;

                public static final double GAIN_THRESHOLD = 6;

                public static final double kp = 0.025;
                public static final double ki = 0;
                public static final double kd = 0;
                public static final double kp1 = 0.05;
                public static final double ki1 = 0;
                public static final double kd1 = 0;
                public static final double kp2 = 0.01;
                public static final double ki2 = 0;
                public static final double kd2 = 0;

                public static final double ArmIZone = 2;

                public static final double FFCONSTANT = 0.04;

                public static final double GEAR_RATIO = 125;
        }

        public static class CoralConstants {
                public static final int MOTOR_ID = 21;
                public static final int CURRENT_LIMIT = 20;
                public static final double INTAKE_VOLTAGE = 1.0;
                public static final double HOLD_PIECE_VOLTAGE = 0.1;
                public static final double HOLD_PIECE_AUTON_VOLTAGE = 0.3;
                public static final double SLOW_SCORE_VOLTAGE = -0.3;

                public static final int COLOR_SENSOR = 27;
                public static final double PROXIMITY = 0.055;
        }

        public static class AlgaeConstants {
                public static final int MOTOR_ID = 22;
                public static final int CURRENT_LIMIT = 20;
                public static final double INTAKE_VOLTAGE = 1.0;
                public static final double OUTTAKE_VOLTAGE = -1.0;
                public static final double HOLD_PIECE_VOLTAGE = 0.7;

                public static final int COLOR_SENSOR = 28;
                public static final double PROXIMITY = 0.15;
        }

        public static class ClimbConstants {
                public static final int CLIMB_MOTOR_ID = 31;
                public static final int FOLLOW_MOTOR_ID = 30;
                public static final int GRAB_MOTOR_ID = 32;
                public static final int RATCHET_SERVO_PORT = 0;

                public static final int CLIMB_CURRENT_LIMIT = 40;

                public static final int START_CLIMB_POSITION = 0;
                public static final int RAISE_CLIMB_POSITION = 40;
                public static final int LOWER_CLIMB_POSITION = -40;

                public static final int CLIMB_POSITION_TOLERANCE = 3;
                public static final double RATCHET_POSITION_TOLERANCE = 0.1;

                public static final double RATCHET_LOCK_POSITION = 0.7;
                public static final double RATCHET_UNLOCK_POSITION = 1;

                public static final double CLIMB_kP = 0.75;
                public static final double CLIMB_kI = 0;
                public static final double CLIMB_kD = 0;

                public static final double COMMAND_WAIT_TIME = 2;
                public static final double LOWER_CLIMB_TIMEOUT = 2.5;
                public static final double LOCK_RATCHET_WAIT_TIME = 0.5;
        }

        public static class LimelightConstants {
                // REEF CAMERA OFFSETS
                public static final double CAMERA_FORWARD_OFFSET = 0.216;
                public static final double CAMERA_SIDE_OFFSET = 0.121;
                public static final double CAMERA_HEIGHT_OFFSET = 0.2404;

                public static final double CAMERA_ROLL_OFFSET = 0.0;
                public static final double CAMERA_PITCH_OFFSET = 0.0;
                public static final double CAMERA_YAW_OFFSET = -17.5;

                public static final double CAMERA2_FORWARD_OFFSET = -0.216;
                public static final double CAMERA2_SIDE_OFFSET = 0.121;
                public static final double CAMERA2_HEIGHT_OFFSET = 0.84990178;

                // CORAL STATION CAMERA OFFSETS
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
                public static final String ALGAE_ALIGN_COMMAND = "VIENNA IS SHORT";
                public static final AprilTagFieldLayout layout = AprilTagFieldLayout
                                .loadField(AprilTagFields.k2025ReefscapeWelded);
                // right is more negative, left is more positive
                public static final double offsetOfArmReef = .3;
                public static final double offsetOfArmReefLeft = .33;
                // right is more negative, left is more positive
                public static final double offsetOfArmStation = -.42;
                public static final double sidewaysOffsetStation = .3;
                public static final double offsetToReef = .52;
                public static final double offsetToStation = .60;
                public static final double reefWidth = 0.33;

                public static final double offsetOfArmAlgae = 0.31;
                public static final double offsetToReefAlgae = 0.53;
                // negative is more right

                // CURRENTLY TESTING WITH THIS LIBRARY THING
                public static final Pose2d RED_PLAYER_STATION_1_CENTER = CalculatePSPoint(layout.getTagPose(1).get(),
                                0);
                public static final Pose2d RED_PLAYER_STATION_1_LEFT = CalculatePSPoint(layout.getTagPose(1).get(),
                                -offsetToStation);
                public static final Pose2d RED_PLAYER_STATION_1_RIGHT = CalculatePSPoint(layout.getTagPose(1).get(),
                                offsetToStation);

                public static final Pose2d RED_PLAYER_STATION_2_CENTER = CalculatePSPoint(layout.getTagPose(2).get(),
                                0);
                public static final Pose2d RED_PLAYER_STATION_2_LEFT = CalculatePSPoint(layout.getTagPose(2).get(),
                                -sidewaysOffsetStation);
                public static final Pose2d RED_PLAYER_STATION_2_RIGHT = CalculatePSPoint(layout.getTagPose(2).get(),
                                sidewaysOffsetStation);

                public static final Pose2d BLUE_PLAYER_STATION_13_CENTER = CalculatePSPoint(layout.getTagPose(13).get(),
                                0);
                public static final Pose2d BLUE_PLAYER_STATION_13_LEFT = CalculatePSPoint(layout.getTagPose(13).get(),
                                -sidewaysOffsetStation);
                public static final Pose2d BLUE_PLAYER_STATION_13_RIGHT = CalculatePSPoint(layout.getTagPose(13).get(),
                                sidewaysOffsetStation);

                public static final Pose2d BLUE_PLAYER_STATION_12_CENTER = CalculatePSPoint(layout.getTagPose(12).get(),
                                0);
                public static final Pose2d BLUE_PLAYER_STATION_12_LEFT = CalculatePSPoint(layout.getTagPose(12).get(),
                                -sidewaysOffsetStation);
                public static final Pose2d BLUE_PLAYER_STATION_12_RIGHT = CalculatePSPoint(layout.getTagPose(12).get(),
                                sidewaysOffsetStation);

                public static final Pose2d RED_REEF_6A = CalculatePoint(layout.getTagPose(6).get(), true);
                public static final Pose2d RED_REEF_6B = CalculatePoint(layout.getTagPose(6).get(), false);
                public static final Pose2d RED_REEF_7A = CalculatePoint(layout.getTagPose(7).get(), true);
                public static final Pose2d RED_REEF_7B = CalculatePoint(layout.getTagPose(7).get(), false);
                public static final Pose2d RED_REEF_8A = CalculatePoint(layout.getTagPose(8).get(), true);
                public static final Pose2d RED_REEF_8B = CalculatePoint(layout.getTagPose(8).get(), false);
                public static final Pose2d RED_REEF_9A = CalculatePoint(layout.getTagPose(9).get(), true);
                public static final Pose2d RED_REEF_9B = CalculatePoint(layout.getTagPose(9).get(), false);
                public static final Pose2d RED_REEF_10A = CalculatePoint(layout.getTagPose(10).get(), true);
                public static final Pose2d RED_REEF_10B = CalculatePoint(layout.getTagPose(10).get(), false);
                public static final Pose2d RED_REEF_11A = CalculatePoint(layout.getTagPose(11).get(), true);
                public static final Pose2d RED_REEF_11B = CalculatePoint(layout.getTagPose(11).get(), false);

                public static final Pose2d BLUE_REEF_17A = CalculatePoint(layout.getTagPose(17).get(), false);
                public static final Pose2d BLUE_REEF_17B = CalculatePoint(layout.getTagPose(17).get(), true);
                public static final Pose2d BLUE_REEF_18A = CalculatePoint(layout.getTagPose(18).get(), false);
                public static final Pose2d BLUE_REEF_18B = CalculatePoint(layout.getTagPose(18).get(), true);
                public static final Pose2d BLUE_REEF_19A = CalculatePoint(layout.getTagPose(19).get(), false);
                public static final Pose2d BLUE_REEF_19B = CalculatePoint(layout.getTagPose(19).get(), true);
                public static final Pose2d BLUE_REEF_20A = CalculatePoint(layout.getTagPose(20).get(), false);
                public static final Pose2d BLUE_REEF_20B = CalculatePoint(layout.getTagPose(20).get(), true);
                public static final Pose2d BLUE_REEF_21A = CalculatePoint(layout.getTagPose(21).get(), false);
                public static final Pose2d BLUE_REEF_21B = CalculatePoint(layout.getTagPose(21).get(), true);
                public static final Pose2d BLUE_REEF_22A = CalculatePoint(layout.getTagPose(22).get(), false);
                public static final Pose2d BLUE_REEF_22B = CalculatePoint(layout.getTagPose(22).get(), true);

                // Don't reorder this list

                public static final List<Pose2d> POSE_LIST = List.of(RED_PLAYER_STATION_1_LEFT,
                                RED_PLAYER_STATION_1_CENTER, RED_PLAYER_STATION_1_RIGHT, RED_PLAYER_STATION_2_LEFT,
                                RED_PLAYER_STATION_2_CENTER, RED_PLAYER_STATION_2_RIGHT,
                                BLUE_PLAYER_STATION_12_LEFT, BLUE_PLAYER_STATION_12_CENTER,
                                BLUE_PLAYER_STATION_12_RIGHT, BLUE_PLAYER_STATION_13_LEFT,
                                BLUE_PLAYER_STATION_13_CENTER, BLUE_PLAYER_STATION_13_RIGHT, RED_REEF_6A, RED_REEF_6B,
                                RED_REEF_7A, RED_REEF_7B, RED_REEF_8A, RED_REEF_8B, RED_REEF_9A, RED_REEF_9B,
                                RED_REEF_10A, RED_REEF_10B, RED_REEF_11A, RED_REEF_11B, BLUE_REEF_17A, BLUE_REEF_17B,
                                BLUE_REEF_18A, BLUE_REEF_18B, BLUE_REEF_19A, BLUE_REEF_19B, BLUE_REEF_20A,
                                BLUE_REEF_20B, BLUE_REEF_21A, BLUE_REEF_21B, BLUE_REEF_22A, BLUE_REEF_22B);

                public static final List<Pose2d> RED_PLAYER_STATION = List.of(RED_PLAYER_STATION_1_LEFT,
                                RED_PLAYER_STATION_1_CENTER, RED_PLAYER_STATION_1_RIGHT, RED_PLAYER_STATION_2_LEFT,
                                RED_PLAYER_STATION_2_CENTER, RED_PLAYER_STATION_2_RIGHT);

                public static final List<Pose2d> BLUE_PLAYER_STATION = List.of(BLUE_PLAYER_STATION_12_LEFT,
                                BLUE_PLAYER_STATION_12_CENTER, BLUE_PLAYER_STATION_12_RIGHT,
                                BLUE_PLAYER_STATION_13_LEFT, BLUE_PLAYER_STATION_13_CENTER,
                                BLUE_PLAYER_STATION_13_RIGHT);

                public static final List<Pose2d> RED_REEF = List.of(RED_REEF_6A, RED_REEF_6B, RED_REEF_7A, RED_REEF_7B,
                                RED_REEF_8A,
                                RED_REEF_8B, RED_REEF_9A, RED_REEF_9B, RED_REEF_10A, RED_REEF_10B, RED_REEF_11A,
                                RED_REEF_11B);

                public static final List<Pose2d> RED_REEF_RIGHT = List.of(RED_REEF_6B, RED_REEF_7B, RED_REEF_8B,
                                RED_REEF_9B,
                                RED_REEF_10B, RED_REEF_11B);

                public static final List<Pose2d> RED_REEF_LEFT = List.of(RED_REEF_6A, RED_REEF_7A, RED_REEF_8A,
                                RED_REEF_9A,
                                RED_REEF_10A, RED_REEF_11A);

                public static final List<Pose2d> BLUE_REEF = List.of(BLUE_REEF_22B, BLUE_REEF_22A, BLUE_REEF_21B,
                                BLUE_REEF_21A,
                                BLUE_REEF_20B, BLUE_REEF_20A, BLUE_REEF_19B, BLUE_REEF_19A, BLUE_REEF_18B,
                                BLUE_REEF_18A, BLUE_REEF_17B,
                                BLUE_REEF_17A);

                public static final List<Pose2d> BLUE_REEF_RIGHT = List.of(BLUE_REEF_22A,
                                BLUE_REEF_21A, BLUE_REEF_20A, BLUE_REEF_19A, BLUE_REEF_18A, BLUE_REEF_17A);

                public static final List<Pose2d> BLUE_REEF_LEFT = List.of(BLUE_REEF_22B,
                                BLUE_REEF_21B, BLUE_REEF_20B, BLUE_REEF_19B, BLUE_REEF_18B, BLUE_REEF_17B);

                public static enum Side {
                        LEFT, RIGHT, NEAREST;
                }

                public static final List<Pose2d> REEF_ALGAE = List.of(
                                CalculateAlgaePoint(layout.getTagPose(6).get()),
                                CalculateAlgaePoint(layout.getTagPose(7).get()),
                                CalculateAlgaePoint(layout.getTagPose(8).get()),
                                CalculateAlgaePoint(layout.getTagPose(9).get()),
                                CalculateAlgaePoint(layout.getTagPose(10).get()),
                                CalculateAlgaePoint(layout.getTagPose(11).get()),
                                CalculateAlgaePoint(layout.getTagPose(17).get()),
                                CalculateAlgaePoint(layout.getTagPose(18).get()),
                                CalculateAlgaePoint(layout.getTagPose(19).get()),
                                CalculateAlgaePoint(layout.getTagPose(20).get()),
                                CalculateAlgaePoint(layout.getTagPose(21).get()),
                                CalculateAlgaePoint(layout.getTagPose(22).get()));
        }
}