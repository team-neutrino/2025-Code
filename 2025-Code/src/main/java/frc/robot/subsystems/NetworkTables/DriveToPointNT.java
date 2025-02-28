package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveToPointNT {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic offsetOfArm = nt.getDoubleTopic("/DriveToPoint/offsetOfArm");
    DoubleTopic offsetToReef = nt.getDoubleTopic("/DriveToPoint/offsetToReef");
    DoubleTopic offsetToStation = nt.getDoubleTopic("/DriveToPoint/offsetToStation");

    final DoublePublisher offsetOfArmPub;
    final DoublePublisher offsetToReefPub;
    final DoublePublisher offsetToStationPub;

}
