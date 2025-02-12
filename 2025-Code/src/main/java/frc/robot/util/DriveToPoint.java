package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveToPoint {

    private static Pose2d m_target = new Pose2d(1.5, 1.2, Rotation2d.fromDegrees(-125));

    public static void setTarget(Pose2d target) {
        m_target = target;
    }

    public double getXVelocity() {
        return (m_target.getX() - Subsystem.swerve.getCurrentPose().getX()) * 2;
    }

    public double getYVelocity() {
        return (m_target.getY() - Subsystem.swerve.getCurrentPose().getY()) * 2;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(m_target.getRotation().getDegrees());
    }
}
