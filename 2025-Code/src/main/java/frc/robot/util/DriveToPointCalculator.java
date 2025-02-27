package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import static frc.robot.Constants.DriveToPoint.*;

public class DriveToPointCalculator {
        public static Pose2d CalculatePoint(Pose3d tagPosition, boolean isA) {
                double reefAngle = tagPosition.getRotation().getAngle();
                double perpendicularReefAngle = reefAngle - Math.toRadians(90);

                double leftRightOffsetX = offsetOfArm * Math.cos(perpendicularReefAngle)
                                + (reefWidth / 2) * Math.cos(perpendicularReefAngle + (isA ? 0 : Math.PI));
                double leftRightOffsetY = offsetOfArm * Math.sin(perpendicularReefAngle)
                                + (reefWidth / 2) * Math.sin(perpendicularReefAngle + (isA ? 0 : Math.PI));

                double x = tagPosition.getX() + offsetToReef * Math.cos(reefAngle) + leftRightOffsetX;
                double y = tagPosition.getY() + offsetToReef * Math.sin(reefAngle) + leftRightOffsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(reefAngle + Math.PI));
        }

        public static Pose2d CalculatePSPoint(Pose3d tagPosition) {
                double stationAngle = tagPosition.getRotation().getAngle();
                double robotAngle = stationAngle + (((Math.PI * 2) - stationAngle) * 2);

                double leftRightOffsetX = offsetOfArm * Math.cos(stationAngle)
                                + (reefWidth / 2) * Math.cos(stationAngle);
                double leftRightOffsetY = offsetOfArm * Math.sin(stationAngle)
                                + (reefWidth / 2) * Math.sin(stationAngle);

                double x = tagPosition.getX() + (offsetToStation * Math.cos(robotAngle)) + leftRightOffsetX;
                double y = tagPosition.getY() + (offsetToStation * Math.sin(robotAngle)) + leftRightOffsetY;

                return new Pose2d(x, y, Rotation2d.fromRadians(robotAngle));
        }
}