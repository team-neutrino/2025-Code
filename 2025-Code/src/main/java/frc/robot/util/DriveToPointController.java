package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.commands.DriveToPointCommand;

import static frc.robot.Constants.SwerveConstants.*;

import java.util.List;

public class DriveToPointController {

    private PIDController m_controlskerX = new PIDController(DRIVE_TO_POINT_P, 0, DRIVE_TO_POINT_D);
    private PIDController m_controlskerY = new PIDController(DRIVE_TO_POINT_P, 0, DRIVE_TO_POINT_D);
    private static Pose2d m_target = new Pose2d(0, 0, new Rotation2d());
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable driveStateTable = nt.getTable("DriveToPoint");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

    public void setTarget(Pose2d target) {
        m_target = target;
        drivePose.set(m_target);
    }

    public Pose2d getTarget() {
        return m_target;
    }

    public double getXVelocity() {
        if (Subsystem.swerve.isAtPoint()) {
            return 0;
        }
        return m_controlskerX.calculate(Subsystem.swerve.getCurrentPose().getX() - m_target.getX());
    }

    public double getYVelocity() {
        if (Subsystem.swerve.isAtPoint()) {
            return 0;
        }
        return m_controlskerY.calculate(Subsystem.swerve.getCurrentPose().getY() - m_target.getY());
    }

    public double getXDistance() {
        return Math.abs(m_target.getX() - Subsystem.swerve.getCurrentPose().getX());
    }

    public double getYDistance() {
        return Math.abs(m_target.getY() - Subsystem.swerve.getCurrentPose().getY());
    }

    public double getStraightLineDist() {
        return Math.hypot(getXDistance(), getYDistance());
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(m_target.getRotation().getDegrees());
    }

    public Pose2d getClosestPoint(List<Pose2d> list) {
        return Subsystem.swerve.getCurrentPose().nearest(list);
    }

    public void setTargetNearest(List<Pose2d> list) {
        setTarget(getClosestPoint(list));
    }
}
