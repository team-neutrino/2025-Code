package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MaxMotionTuner {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic MaxVelocity;
    DoubleTopic MaxAcceleration;
    DoubleTopic AllowedError;
    DoublePublisher MaxVelocity_Pub;
    DoublePublisher MaxAcceleration_Pub;
    DoublePublisher AllowedError_Pub;
    DoubleSubscriber MaxVelocity_Sub;
    DoubleSubscriber MaxAcceleration_Sub;
    DoubleSubscriber AllowedError_Sub;

    public MaxMotionTuner(String subsystemName) {

        MaxVelocity = nt.getDoubleTopic("/" + subsystemName + "/MaxVelocity");
        MaxAcceleration = nt.getDoubleTopic("/" + subsystemName + "/MaxAcceleration");
        AllowedError = nt.getDoubleTopic("/" + subsystemName + "/AllowedError");

        MaxVelocity_Pub = MaxVelocity.publish();
        MaxVelocity_Pub.setDefault(0.0);
        MaxAcceleration_Pub = MaxAcceleration.publish();
        MaxAcceleration_Pub.setDefault(0.0);
        AllowedError_Pub = AllowedError.publish();
        AllowedError_Pub.setDefault(0.0);

        MaxVelocity_Sub = MaxVelocity.subscribe(0);
        MaxAcceleration_Sub = MaxAcceleration.subscribe(0);
        AllowedError_Sub = AllowedError.subscribe(0);
    }

    public double getMaxVelocity() {
        return MaxVelocity_Sub.get();
    }

    public double getMaxAcceleration() {
        return MaxAcceleration_Sub.get();
    }

    public double getAllowedError() {
        return AllowedError_Sub.get();
    }

    public void setMaxVelocity(double maxVelocity) {
        MaxVelocity_Pub.set(maxVelocity);
    }

    public void setMaxAcceleration(double maxAcceleration) {
        MaxAcceleration_Pub.set(maxAcceleration);
    }

    public void setAllowedError(double allowedError) {
        AllowedError_Pub.set(allowedError);
    }

    public boolean isDifferentValues(double previousMaxVelocity, double previousMaxAcceleration,
            double previousAllowableError) {
        return getMaxVelocity() != previousMaxVelocity || getMaxAcceleration() != previousMaxAcceleration ||
                getAllowedError() != previousAllowableError;
    }

    public void periodic() {
    }
}
