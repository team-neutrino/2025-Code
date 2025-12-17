package frc.robot.util;

public class CoralDetectionMath {
    private static final int TOTAL_PIXEL_HEIGHT = 4080; 
    private static final int TOTAL_PIXEL_WIDTH = 3072;
    private static final double CAMERA_HEIGHT_FOV = 1.22; 
    private static final double CAMERA_WIDTH_FOV = 0.96;
    private static final double ANGLE_TO_MIDDLE_Y = 0.61;
    private static final double CAMERA_HEIGHT = 0.88;

    private static final double EXPECTED_X = 0.32;
    private static final double EXPECTED_Y = 0.9;

    public static void calculateCoralPos(int pixelX, int pixelY) {
        double angleToCoralY = calculateAngleToCoralY(pixelY);
        System.out.println("angle to coral Y: " + angleToCoralY);
        double distanceFromCameraY = CAMERA_HEIGHT * Math.tan(angleToCoralY);

        double angleToCoralX = calculateAngleToCoralX(pixelX);
        System.out.println("angle to coral X: " + angleToCoralX);
        double distanceFromCameraX = distanceFromCameraY * Math.tan(angleToCoralX);

        System.out.println("X: " + distanceFromCameraX);
        System.out.println("Y: " + distanceFromCameraY);
    }

    private static double calculateAngleToCoralY(int pixelY) {
        double pixelToAngleRatio = CAMERA_HEIGHT_FOV / TOTAL_PIXEL_HEIGHT;
        System.out.println("pixel to angle Y: " + pixelToAngleRatio);
        double middleToCoralAngle = pixelY * pixelToAngleRatio;
        System.out.println("middle to coral angle Y: " + middleToCoralAngle);
        return middleToCoralAngle + ANGLE_TO_MIDDLE_Y;
    }

    private static double calculateAngleToCoralX(int pixelX) {
        double pixelToAngleRatio = CAMERA_WIDTH_FOV / TOTAL_PIXEL_WIDTH;
        System.out.println("pixel to angle X: " + pixelToAngleRatio);
        return pixelX * pixelToAngleRatio;
    }

    public static void calculateCameraFov(int pixelX, int pixelY) 
    {   
        System.out.println("pixelY: " + pixelY);
        double fractionY = ((double)pixelY / (double)TOTAL_PIXEL_HEIGHT);
        System.out.println("fractionY: " + fractionY);
        double heightFov = (Math.atan(EXPECTED_Y / CAMERA_HEIGHT) - ANGLE_TO_MIDDLE_Y) / fractionY;

        double fractionX = (double)pixelX / (double)TOTAL_PIXEL_WIDTH;
        System.out.println("fractionX: " + fractionX);
        double widthFov = (Math.atan(EXPECTED_X / EXPECTED_Y)) / fractionX;

        double widthFov2 = TOTAL_PIXEL_WIDTH * Math.atan(EXPECTED_X / EXPECTED_Y) / pixelX * 57.2;

        System.out.println("expected height fov: " + heightFov);
        System.out.println("expected width fov: " + widthFov);
        System.out.println("expected width fov2: " + widthFov2);

    }
}
