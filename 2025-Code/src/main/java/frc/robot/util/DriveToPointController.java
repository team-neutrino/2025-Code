package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.Constants.SwerveConstants.*;

import java.util.List;

public class DriveToPointController {

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
        double xVelocity = Subsystem.arm.isAtIntake()
                ? (applyDeadband((m_target.getX() - Subsystem.swerve.getCurrentPose().getX()), 0.00000001, 70.0)
                        * DRIVE_TO_POINT_INTAKE_P)
                : (applyDeadband((m_target.getX() - Subsystem.swerve.getCurrentPose().getX()), 0.00000001, 70.0)
                        * DRIVE_TO_POINT_P);
        return xVelocity;
    }

    public double getYVelocity() {
        double yVelocity = Subsystem.arm.isAtIntake()
                ? (applyDeadband((m_target.getY() - Subsystem.swerve.getCurrentPose().getY()), 0.000000001, 70.0)
                        * DRIVE_TO_POINT_INTAKE_P)
                : (applyDeadband((m_target.getY() - Subsystem.swerve.getCurrentPose().getY()), 0.000000001, 70.0)
                        * DRIVE_TO_POINT_P);
        return yVelocity;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(m_target.getRotation().getDegrees());
    }

    public Pose2d getClosestPoint(List<Pose2d> list) {
        return Subsystem.swerve.getCurrentPose().nearest(list);
    }

    public void setTargetNearest(List<Pose2d> list) {
        m_target = getClosestPoint(list);
    }
}
