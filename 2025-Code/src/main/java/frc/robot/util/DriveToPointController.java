package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.Constants.DriveToPoint.*;
import static frc.robot.Constants.SwerveConstants.*;

import java.util.List;

public class DriveToPointController {

    private static Pose2d m_target = new Pose2d(0, 0, new Rotation2d());

    public void setTarget(Pose2d target) {
        m_target = target;
    }

    public Pose2d getTarget() {
        return m_target;
    }

    public double getXVelocity() {
        return (applyDeadband((m_target.getX() - Subsystem.swerve.getCurrentPose().getX()), 0.01, 70.0)
                * DRIVE_TO_POINT_P);
    }

    public double getYVelocity() {
        return (applyDeadband((m_target.getY() - Subsystem.swerve.getCurrentPose().getY()), 0.01, 70.0)
                * DRIVE_TO_POINT_P);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(m_target.getRotation().getDegrees() + 180);
    }

    public Pose2d getClosestPoint(List<Pose2d> list) {
        return Subsystem.swerve.getCurrentPose().nearest(list);
    }

    public void setTargetNearest(List<Pose2d> list) {
        m_target = getClosestPoint(list);
    }
}
