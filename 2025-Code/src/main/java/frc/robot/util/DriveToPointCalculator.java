package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

import static frc.robot.Constants.DriveToPoint.*;

public class DriveToPointCalculator {
        public static Pose2d CalculatePoint(Pose3d tagPosition, boolean isA) {
                double reefAngle = tagPosition.getRotation().getAngle();
                double perpendicularReefAngle = reefAngle - Math.toRadians(90);

                double leftRightOffsetX = OFFSET_ARM_REEF
                                * Math.cos(perpendicularReefAngle)
                                + (REEF_WIDTH / 2) * Math.cos(perpendicularReefAngle + (isA ? 0 : Math.PI));
                double leftRightOffsetY = OFFSET_ARM_REEF
                                * Math.sin(perpendicularReefAngle)
                                + (REEF_WIDTH / 2) * Math.sin(perpendicularReefAngle + (isA ? 0 : Math.PI));

                double x = tagPosition.getX() + OFFSET_TO_REEF * Math.cos(reefAngle) + leftRightOffsetX;
                double y = tagPosition.getY() + OFFSET_TO_REEF * Math.sin(reefAngle) + leftRightOffsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(reefAngle + Math.PI));
        }

        public static Pose2d CalculatePSPoint(Pose3d tagPosition, double sidewaysOffset) {
                double stationAngle = tagPosition.getRotation().getAngle();
                double perpendicularReefAngle = stationAngle - Math.toRadians(90);

                double cos = Math.cos(perpendicularReefAngle), sin = Math.sin(perpendicularReefAngle);
                double leftRightOffsetX = (OFFSET_ARM_STATION * cos) + ((REEF_WIDTH / 2) * cos)
                                + (sidewaysOffset * cos);
                double leftRightOffsetY = (OFFSET_ARM_STATION * sin) + ((REEF_WIDTH / 2) * sin)
                                + (sidewaysOffset * sin);

                double x = tagPosition.getX() + (OFFSET_TO_STATION * Math.cos(stationAngle)) + leftRightOffsetX;
                double y = tagPosition.getY() + (OFFSET_TO_STATION * Math.sin(stationAngle)) + leftRightOffsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(stationAngle));
        }

        public static Pose2d CalculatePSSafePoint(Pose3d tagPosition, double sidewaysOffset) {
                double stationAngle = tagPosition.getRotation().getAngle();

                double x = tagPosition.getX() + (SAFE_OFFSET_TO_STATION * Math.cos(stationAngle));
                double y = tagPosition.getY() + (SAFE_OFFSET_TO_STATION * Math.sin(stationAngle));

                return new Pose2d(x, y, Rotation2d.fromRadians(stationAngle));
        }

        public static Pose2d CalculateAlgaePoint(Pose3d tagPosition) {
                double reefAngle = tagPosition.getRotation().getAngle();
                double perpendicularReefAngle = reefAngle - Math.toRadians(90);

                double offsetX = (OFFSET_ARM_ALGAE)
                                * Math.cos(perpendicularReefAngle);
                double offsetY = (OFFSET_ARM_ALGAE)
                                * Math.sin(perpendicularReefAngle);

                double x = tagPosition.getX() + OFFSET_REEF_ALGAE * Math.cos(reefAngle) + offsetX;
                double y = tagPosition.getY() + OFFSET_REEF_ALGAE * Math.sin(reefAngle) + offsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(reefAngle + Math.PI));
        }

        public static Pose2d CalculateBargePoint(Pose3d tagPosition) {
                double bargeAngle = tagPosition.getRotation().getMeasureZ().in(Units.Radians);
                double offsetX = OFFSET_TO_BARGE_X;
                double offsetY = OFFSET_TO_BARGE_Y;
                if (bargeAngle == 0) {
                        offsetX *= -1;
                        offsetY *= -1;
                }
                double x = tagPosition.getX() - offsetX;
                double y = tagPosition.getY() - offsetY;
                return new Pose2d(x, y, Rotation2d.fromRadians(bargeAngle));
        }
}