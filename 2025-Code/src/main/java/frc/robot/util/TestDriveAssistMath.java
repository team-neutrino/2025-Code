// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class TestDriveAssistMath {
    public static void main(String[] args) {

    }

    // for yaw, left is positive and right is negative
    private static Translation2d testing(double yaw, double dist, int id) {
        double hexagonAngle = id % 7 == 6 ? 300 : (id % 7) * 60;
        double triangle1Angle = Math.toRadians(hexagonAngle - yaw);
        double error = Math.abs(Math.sin(triangle1Angle) * dist);

        // to field coordinate
        double triangle2Angle = Integer.MAX_VALUE;
        switch (id) {
            case 7:
                triangle2Angle = 0;
                break;
            case 8:
                triangle2Angle = 3; // TODO: solve other cases
                break;
        }

        double xError = error * Math.cos(triangle2Angle);
        double yError = error * Math.sin(triangle1Angle);

        return new Translation2d(xError, yError);
    }

    private static Translation2d current(double yaw, double dist, int id) {
        int tx = 0;
        // int pov = controller.getHID().getPOV();
        // point of interest offset
        // POIoffset = (pov == 270 ? -SwerveConstants.REEF_OFFSET : pov == 90 ?
        // REEF_OFFSET : POIoffset);
        // limelight.setPointOfInterest(0, POIoffset);
        int idMod = id % 7;

        double reefSideAngle = idMod == 6 ? 300 : idMod * 60;
        reefSideAngle = 0;

        double triangle1angle = Math
                .toRadians(yaw - tx) - reefSideAngle;
        double limelightTagToRobot = dist;
        double targetError = (limelightTagToRobot) * Math.sin(triangle1angle);

        double xVel = targetError * Math.sin(reefSideAngle);
        double yVel = targetError * Math.cos(reefSideAngle);

        return new Translation2d(xVel, yVel);
    }
}
