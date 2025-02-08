package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.SwerveRequestStash;

import static frc.robot.Constants.SwerveConstants.MAX_SPEED;
import static frc.robot.util.Subsystem.*;

public class AprilTagDriveAssist extends DriveAssistCom {
    private com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle req = SwerveRequestStash.driveAssist;

    public AprilTagDriveAssist(CommandXboxController p_controller) {
        super(p_controller);
    }

    @Override
    public void execute() {
        if (!limelight.getTv() || exitExecute()) {
            swerve.setControl(req.withVelocityX(0)
                    .withVelocityY(0));
            return;
        }
        double x = limelight.getRobotToTag().getX();
        double y = limelight.getRobotToTag().getY();
        Translation2d velocities = getVelocities();
        swerve.setIsAligned(isAligned());
        Rotation2d angle = Rotation2d.fromDegrees(swerve.getYawDegrees() - limelight.getTx());
        swerve.setControl(req.withVelocityX(velocities.getX() + -m_controller.getLeftY() * MAX_SPEED)
                .withVelocityY(velocities.getY() + -m_controller.getLeftX() * MAX_SPEED).withTargetDirection(angle));
    }

    private Translation2d getVelocities() {
        getAprilTagFieldRelativeDistances();
        return null;
    }

    private Translation2d getAprilTagFieldRelativeDistances() {
        return null;
    }
}