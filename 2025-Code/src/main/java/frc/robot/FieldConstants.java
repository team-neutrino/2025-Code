package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * All values for the pose3d are in inches and the values for the rotation3d are
 * in radians!
 */
public final class FieldConstants {

        /**
         * null first entry because arrays start at 0 while apriltag ids start at 1
         */
        public static final Pose3d[] APRIL_TAG_POSITIONS = { null,
                        new Pose3d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.8),
                                        Units.inchesToMeters(58.50),
                                        new Rotation3d(Math.toRadians(126), 0, 0)),
                        new Pose3d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20),
                                        Units.inchesToMeters(58.5),
                                        new Rotation3d(Math.toRadians(234), 0, 0)),
                        new Pose3d(Units.inchesToMeters(455.15), Units.inchesToMeters(317.15),
                                        Units.inchesToMeters(51.25),
                                        new Rotation3d(Math.toRadians(270), 0, 0)),
                        new Pose3d(Units.inchesToMeters(365.20), Units.inchesToMeters(241.64),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(0, 0, Math.toRadians(30))),
                        new Pose3d(Units.inchesToMeters(365.20), Units.inchesToMeters(75.39),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(0, 0, Math.toRadians(30))),
                        new Pose3d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(300), 0, 0)),
                        new Pose3d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(0, 0, 0)),
                        new Pose3d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(60), 0, 0)),
                        new Pose3d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(120), 0, 0)),
                        new Pose3d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.5),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(180), 0, 0)),
                        new Pose3d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(240), 0, 0)),
                        new Pose3d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.8),
                                        Units.inchesToMeters(58.5),
                                        new Rotation3d(Math.toRadians(54), 0, 0)),
                        new Pose3d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.8),
                                        Units.inchesToMeters(58.5),
                                        new Rotation3d(Math.toRadians(306), 0, 0)),
                        new Pose3d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(Math.toRadians(180), 0, 0)),
                        new Pose3d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(Math.toRadians(180), 0, 0)),
                        new Pose3d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15),
                                        Units.inchesToMeters(73.54),
                                        new Rotation3d(Math.toRadians(90), 0, 0)),
                        new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(240), 0, 0)),
                        new Pose3d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(180), 0, 0)),
                        new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(120), 0, 0)),
                        new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(60), 0, 0)),
                        new Pose3d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(0), 0, 0)),
                        new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17),
                                        Units.inchesToMeters(12.13),
                                        new Rotation3d(Math.toRadians(300), 0, 0)) };
}