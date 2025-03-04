package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.DriveToPoint.*;

public class DriveToPointCalculator {
        public static Pose2d CalculatePoint(Pose3d tagPosition, boolean isA) {
                double reefAngle = tagPosition.getRotation().getAngle();
                double perpendicularReefAngle = reefAngle - Math.toRadians(90);

                double leftRightOffsetX = (isA ? offsetOfArmReefLeft : offsetOfArmReef)
                                * Math.cos(perpendicularReefAngle)
                                + (reefWidth / 2) * Math.cos(perpendicularReefAngle + (isA ? 0 : Math.PI));
                double leftRightOffsetY = (isA ? offsetOfArmReefLeft : offsetOfArmReef)
                                * Math.sin(perpendicularReefAngle)
                                + (reefWidth / 2) * Math.sin(perpendicularReefAngle + (isA ? 0 : Math.PI));

                double x = tagPosition.getX() + offsetToReef * Math.cos(reefAngle) + leftRightOffsetX;
                double y = tagPosition.getY() + offsetToReef * Math.sin(reefAngle) + leftRightOffsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(reefAngle + Math.PI));
        }

        public static Pose2d CalculatePSPoint(Pose3d tagPosition) {
                double stationAngle = tagPosition.getRotation().getAngle();
                double perpendicularReefAngle = stationAngle - Math.toRadians(90);

                double leftRightOffsetX = offsetOfArmStation * Math.cos(perpendicularReefAngle)
                                + (reefWidth / 2) * Math.cos(perpendicularReefAngle);
                double leftRightOffsetY = offsetOfArmStation * Math.sin(perpendicularReefAngle)
                                + (reefWidth / 2) * Math.sin(perpendicularReefAngle);

                double x = tagPosition.getX() + (offsetToStation * Math.cos(stationAngle)) + leftRightOffsetX;
                double y = tagPosition.getY() + (offsetToStation * Math.sin(stationAngle)) + leftRightOffsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(stationAngle));
        }

        public static Pose2d CalculateAlgaePoint(Pose3d tagPosition) {
                double reefAngle = tagPosition.getRotation().getAngle();
                double perpendicularReefAngle = reefAngle - Math.toRadians(90);

                double offsetX = (offsetOfArmAlgae)
                                * Math.cos(perpendicularReefAngle);
                double offsetY = (offsetOfArmAlgae)
                                * Math.sin(perpendicularReefAngle);

                double x = tagPosition.getX() + offsetToReefAlgae * Math.cos(reefAngle) + offsetX;
                double y = tagPosition.getY() + offsetToReefAlgae * Math.sin(reefAngle) + offsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(reefAngle + Math.PI));
        }
}