// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class TestDriveAssistMath {
    private static final int DIST_TESTING_VALUE = 2;

    public static void main(String[] args) {
        System.out.println("REMEMBER OUTPUT IS IN FIELD COORDINATES!!! LEFT IS POSITIVE Y AND UP IS POSITIVE X!!!");
        // case 10 tests (head on): PASSED + audacious
        // System.out.println(testing(-30, DIST_TESTING_VALUE, 10));
        // System.out.println(testing(65, DIST_TESTING_VALUE, 10));
        // case 7 tests (backwards, head on): PASSED + audacious (edge case added)
        // System.out.println(testing(-160, DIST_TESTING_VALUE, 7));
        // System.out.println(testing(130, DIST_TESTING_VALUE, 7));
        // case 8 tests: PASSED
        // System.out.println(testing(-170, DIST_TESTING_VALUE, 8));
        // System.out.println(testing(-87, DIST_TESTING_VALUE, 8));
        // case 9 tests: PASSED
        // System.out.println(testing(-120, DIST_TESTING_VALUE, 9));
        // System.out.println(testing(-2, DIST_TESTING_VALUE, 9));
        // case 11 tests: PASSED + audacious
        // System.out.println(testing(52, DIST_TESTING_VALUE, 11));
        // System.out.println(testing(90, DIST_TESTING_VALUE, 11));
        // case 6 tests: PASSED + audacious
        // System.out.println(testing(140, DIST_TESTING_VALUE, 6));
        // System.out.println(testing(87.5, DIST_TESTING_VALUE, 6));
        // intrinsic edge case testing:
        System.out.println(testing(2, DIST_TESTING_VALUE, 9));
        System.out.println(testing(-5, DIST_TESTING_VALUE, 11));
    }

    /**
     * robot yaw is positive to the left and negative to the right
     * <p>
     * this method assumes that "yaw" value is the perfectly-autoaligned angle and
     * is -180->180
     */
    private static Translation2d testing(double yaw, double dist, int id) {
        System.out.println("id and yaw: " + id + ", " + yaw);

        double hexagonAngle = Integer.MAX_VALUE;
        switch (id) {
            case 10:
                hexagonAngle = 0;
                break;
            case 9:
                hexagonAngle = -60;
                break;
            case 8:
                hexagonAngle = -120;
                break;
            case 7: // special ternary for AUDACIOUS testing! (also makes normal calculation make
                    // more sense, although it is functionally irrelevant for that)
                hexagonAngle = yaw > 0 ? 180 : -180;
                break;
            case 11:
                hexagonAngle = 60;
                break;
            case 6:
                hexagonAngle = 120;
                break;
        }
        // hexagonAngle - yaw works out in a rather beautiful manner that as a result is
        // too complex to explain in a comment
        // TODO: DEAL WITH EDGE CASES
        double triangle1Angle = Math.toRadians(hexagonAngle - yaw);
        double error = Math.abs(Math.sin(triangle1Angle) * dist);

        // audacious appears to fully work - TODO: DEAL WITH EDGE CASES
        double audaciousTri2Angle = hexagonAngle + (triangle1Angle > 0 ? 180 : 0);
        // id 7 AND negative yaw edge case: incorrectly gets -180 instead of 180, negate
        // to correct (this doesn't affect the positive yaw case, in which it gets 360)
        audaciousTri2Angle = id == 7 ? -audaciousTri2Angle : audaciousTri2Angle;

        // use yaw and the fact that the robot will be at a known heading when it is at
        // the target position to determine what quadrant the field-centric triangle is
        // in and therefore the negation status of the x and y error.
        double triangle2Angle = Math.PI / 3; // quadrant 1; minimum possible value
        double autoAlignedYaw = yaw - 0; // "0" will be tx in regular code
        switch (id) {
            case 7, 10:
                // already aligned with field coordinate plane; just use sign of yaw to
                // determine direction
                triangle2Angle = autoAlignedYaw > 0 ? 0 : Math.PI;
                break;
            case 8:
                // yaw will be -120 when at destination, less than that if on left of tag, and
                // greater if on right
                // quadrants 1 and 3 (plus sign rollover case)
                triangle2Angle += (autoAlignedYaw >= 0 || autoAlignedYaw < -120) ? 0 : Math.PI;
                break;
            case 9:
                // yaw will be -60 when when at destination, less than that if on left of tag,
                // and greater if on right
                // quadrants 2 and 4 (sign rollover case intrinsically dealt with)
                triangle2Angle += autoAlignedYaw > -60 ? (Math.PI * 4) / 3 : Math.PI / 3;
                break;
            case 11:
                // yaw will be 60 when at destination, greater than that if on right of tag, and
                // less if on left
                // quadrants 3 and 1 (sign rollover case intrinsically dealt with)
                triangle2Angle += autoAlignedYaw < 60 ? Math.PI : 0;
                break;
            case 6: // weird apriltag ID, dw about it
                // yaw will be 120 when at destination, greater than that if on right of tag,
                // and less if on left
                // quadrants 4 and 2 (plus sign rollover case)
                triangle2Angle += (autoAlignedYaw <= 0 || autoAlignedYaw > 120) ? Math.PI / 3 : (Math.PI * 4) / 3;
                break;
        }
        System.out.println("mundane: " + triangle2Angle + ", audacious: " + audaciousTri2Angle);

        // wpilb y axis is inverted
        double yError = -(error * Math.cos(triangle2Angle));
        double xError = error * Math.sin(triangle2Angle);

        return new Translation2d(xError, yError);
    }

    /**
     * holding the old version in case I need it for some reason
     */
    private Translation2d getFieldRelativeDistances() {
        // int id = limelight.getID();
        // int pov = controller.getHID().getPOV();
        // // point of interest offset
        // POIoffset = (pov == 270 ? -SwerveConstants.REEF_OFFSET : pov == 90 ?
        // REEF_OFFSET : POIoffset);
        // limelight.setPointOfInterest(0, POIoffset);
        // int idMod = id % 7;

        // // angle the reef side makes with the field-plane
        // double reefSideAngle = idMod == 6 ? 300 : idMod * 60;
        // // reefSideAngle = Math.PI / 3;
        // reefSideAngle = 0;

        // // angle of the robot-reef-target right triangle
        // double triangle1angle = Math
        // .toRadians(swerve.getYawDegrees() - limelight.getTx()) - reefSideAngle;
        // System.out.println(Math.toDegrees(triangle1angle));
        // // hypotenuse of above triangle
        // double limelightTagToRobot = limelight.getDistanceFromPrimaryTarget();
        // double targetError = (limelightTagToRobot) * Math.sin(triangle1angle);

        // return new Translation2d(targetError * Math.sin(reefSideAngle), targetError *
        // Math.cos(reefSideAngle));
        return null;
    }
}
