package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.DriveToPoint.*;

public class DriveToPointCalculator {
        public static Pose2d CalculatePoint(Pose3d tagPosition, boolean isA) {
                double reefAngle = tagPosition.getRotation().getAngle();
                double perpendicularReefAngle = reefAngle - Math.toRadians(90);

                double leftRightOffsetX = (isA ? OFFSET_OF_ARM_REEF_LEFT : OFFSET_OF_ARM_REEF)
                                * Math.cos(perpendicularReefAngle)
                                + (REEF_WIDTH / 2) * Math.cos(perpendicularReefAngle + (isA ? 0 : Math.PI));
                double leftRightOffsetY = (isA ? OFFSET_OF_ARM_REEF_LEFT : OFFSET_OF_ARM_REEF)
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
                double leftRightOffsetX = (offsetOfArmStation * cos) + ((reefWidth / 2) * cos) + (sidewaysOffset * cos);
                double leftRightOffsetY = (offsetOfArmStation * sin) + ((reefWidth / 2) * sin) + (sidewaysOffset * sin);

                double x = tagPosition.getX() + (OFFSET_TO_STATION * Math.cos(stationAngle)) + leftRightOffsetX;
                double y = tagPosition.getY() + (OFFSET_TO_STATION * Math.sin(stationAngle)) + leftRightOffsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(stationAngle));
        }

        public static Pose2d CalculateAlgaePoint(Pose3d tagPosition) {
                double reefAngle = tagPosition.getRotation().getAngle();
                double perpendicularReefAngle = reefAngle - Math.toRadians(90);

                double offsetX = (OFFSET_OF_ARM_ALGAE)
                                * Math.cos(perpendicularReefAngle);
                double offsetY = (OFFSET_OF_ARM_ALGAE)
                                * Math.sin(perpendicularReefAngle);

                double x = tagPosition.getX() + OFFSET_TO_REEF_ALGAE * Math.cos(reefAngle) + offsetX;
                double y = tagPosition.getY() + OFFSET_TO_REEF_ALGAE * Math.sin(reefAngle) + offsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(reefAngle + Math.PI));
        }
}