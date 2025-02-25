package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveToPointCalculator {
    public static Pose2d CalculatePoint(Pose3d tagPosition, double offsetFromReef, double offsetOfArm,
            double reefWidth, boolean isA) {
        double reefAngle = tagPosition.getRotation().getAngle();
        double perpendicularReefAngle = reefAngle - Math.toRadians(90);

        double leftRightOffsetX = offsetOfArm * Math.cos(perpendicularReefAngle)
                + (reefWidth / 2) * Math.cos(perpendicularReefAngle + (isA ? 0 : Math.PI));
        double leftRightOffsetY = offsetOfArm * Math.sin(perpendicularReefAngle)
                + (reefWidth / 2) * Math.sin(perpendicularReefAngle + (isA ? 0 : Math.PI));

        double x = tagPosition.getX() + offsetFromReef * Math.cos(reefAngle) + leftRightOffsetX;
        double y = tagPosition.getY() + offsetFromReef * Math.sin(reefAngle) + leftRightOffsetY;

        return new Pose2d(x, y, Rotation2d.fromRadians(reefAngle + Math.PI));
    }
}