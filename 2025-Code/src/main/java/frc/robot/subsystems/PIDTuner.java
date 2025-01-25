package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Subsystem;

public class PIDTuner {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    String subsystem;
    DoubleTopic P;
    DoubleTopic I;
    DoubleTopic D;
    DoublePublisher P_Pub;
    DoublePublisher I_Pub;
    DoublePublisher D_Pub;
    DoubleSubscriber P_Sub;
    DoubleSubscriber I_Sub;
    DoubleSubscriber D_Sub;

    public PIDTuner(String subsystemName) {
        subsystem = subsystemName;

        P = nt.getDoubleTopic("/" + subsystem + "/P");
        I = nt.getDoubleTopic("/" + subsystem + "/I");
        D = nt.getDoubleTopic("/" + subsystem + "/D");

        P_Pub = P.publish();
        P_Pub.setDefault(0.0);
        I_Pub = I.publish();
        I_Pub.setDefault(0.0);
        D_Pub = D.publish();
        D_Pub.setDefault(0.0);

        P_Sub = P.subscribe(0);
        I_Sub = I.subscribe(0);
        D_Sub = D.subscribe(0);
    }

    public double getP() {
        return P_Sub.get();
    }

    public double getI() {
        return I_Sub.get();
    }

    public double getD() {
        return D_Sub.get();
    }

    public void setP(double p) {
        P_Pub.set(p);
    }

    public void setI(double i) {
        I_Pub.set(i);
    }

    public void setD(double d) {
        D_Pub.set(d);
    }

    public void periodic() {
    }
}
