package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;

public class Pose2DController {
    Swerve m_swerve;

    double m_kP = 2.0;
    double m_max = 12.0;
    Pose2d m_target = new Pose2d(1.0, 7.0, Rotation2d.kZero);

    public Pose2DController() {
        m_swerve = Subsystem.swerve;
    }

    public double x() {
        double error = m_target.getX() - m_swerve.getCurrentPose().getX();
        return Math.min(Math.max(m_kP * error, -m_max), m_max);
    }

    public double y() {
        double error = m_target.getY() - m_swerve.getCurrentPose().getY();
        return Math.min(Math.max(m_kP * error, -m_max), m_max);
    }

    public Rotation2d theta() {
        return Rotation2d.kZero;
    }

}
