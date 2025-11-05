package frc.robot.util;

public class CoralDetectionMath {
    private static final int TOTAL_PIXEL_HEIGHT = 4032;
    private static final int TOTAL_PIXEL_WIDTH = 3024;
    private static final double CAMERA_HEIGHT_FOV = 1.38;
    private static final double CAMERA_WIDTH_FOV = 1.38;
    private static final double ANGLE_TO_MIDDLE_X = 0;
    private static final double ANGLE_TO_MIDDLE_Y = 0.698;
    private static final double CAMERA_HEIGHT = 1.31;

    public static void calculateCoralPos(int pixelX, int pixelY) {
        double angleToCoralY = calculateAngleToCoralY(pixelY);
        System.out.println("angle to coral Y: " + angleToCoralY);
        double distanceFromCameraY = CAMERA_HEIGHT * Math.tan(angleToCoralY);

        double angleToCoralX = calculateAngleToCoralX(pixelX);
        double distanceFromCameraX = distanceFromCameraY * Math.tan(angleToCoralX);

        System.out.println("X: " + distanceFromCameraX);
        System.out.println("Y: " + distanceFromCameraY);
    }

    private static double calculateAngleToCoralY(int pixelY) {
        double pixelToAngleRatio = CAMERA_HEIGHT_FOV / TOTAL_PIXEL_HEIGHT;
        System.out.println("pixel to angle Y: " + pixelToAngleRatio);
        double middleToCoralAngle = pixelY * pixelToAngleRatio;
        System.out.println("middle to coral angle: " + middleToCoralAngle);
        return middleToCoralAngle + ANGLE_TO_MIDDLE_Y;
    }

    private static double calculateAngleToCoralX(int pixelX) {
        double pixelToAngleRatio = CAMERA_WIDTH_FOV / TOTAL_PIXEL_WIDTH;
        double middleToCoralAngle = pixelX * pixelToAngleRatio;
        return middleToCoralAngle + ANGLE_TO_MIDDLE_X;
    }
}
