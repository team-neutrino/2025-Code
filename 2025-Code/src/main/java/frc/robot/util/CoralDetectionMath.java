package frc.robot.util;

public class CoralDetectionMath {
    private static final int TOTAL_PIXEL_HEIGHT = 4080; //720
    private static final int TOTAL_PIXEL_WIDTH = 3072; //1280
    private static final double CAMERA_HEIGHT_FOV = 1.43; //0.87
    private static final double CAMERA_WIDTH_FOV = 1.43; //0.87
    private static final double ANGLE_TO_MIDDLE_X = 0;
    private static final double ANGLE_TO_MIDDLE_Y = 1; // 0.78
    private static final double CAMERA_HEIGHT = 0.927;

    private static final double EXPECTED_X = -0.267; //0.00063
    private static final double EXPECTED_Y = 0.48; //0.927

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
        System.out.println("middle to coral angle Y: " + middleToCoralAngle);
        return middleToCoralAngle + ANGLE_TO_MIDDLE_Y;
    }

    private static double calculateAngleToCoralX(int pixelX) {
        double pixelToAngleRatio = CAMERA_WIDTH_FOV / TOTAL_PIXEL_WIDTH;
        System.out.println("pixel to angle X: " + pixelToAngleRatio);
        double middleToCoralAngle = pixelX * pixelToAngleRatio;
        System.out.println("middle to coral angle X: " + middleToCoralAngle);
        return middleToCoralAngle + ANGLE_TO_MIDDLE_X;
    }

    public static void calculateCameraFov(int pixelX, int pixelY) 
    {   
        System.out.println("pixelY: " + pixelY);
        double fractionY = ((double)pixelY / (double)TOTAL_PIXEL_HEIGHT);
        System.out.println("fractionY: " + fractionY);
        double heightFov = (Math.atan(EXPECTED_Y / CAMERA_HEIGHT) - ANGLE_TO_MIDDLE_Y) / fractionY;

        double fractionX = (double)pixelX / (double)TOTAL_PIXEL_WIDTH;
        System.out.println("fractionX: " + fractionX);
        double widthFov = (Math.atan(EXPECTED_X / EXPECTED_Y) - ANGLE_TO_MIDDLE_X) / fractionX;

        double widthFov2 = TOTAL_PIXEL_WIDTH * Math.atan(EXPECTED_X / EXPECTED_Y) / pixelX * 57.2;

        System.out.println("expected height fov: " + heightFov);
        System.out.println("expected width fov: " + widthFov);
        System.out.println("expected width fov2: " + widthFov2);

    }
}
