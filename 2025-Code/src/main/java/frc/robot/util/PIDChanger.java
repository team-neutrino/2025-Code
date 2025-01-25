package frc.robot.util;

import java.util.concurrent.Flow.Subscriber;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.Constants;

public class PIDChanger {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic ArmP = nt.getDoubleTopic("/arm/P");
    DoubleTopic ArmI = nt.getDoubleTopic("/arm/I");
    DoubleTopic ArmD = nt.getDoubleTopic("/arm/D");
    DoubleTopic ElevatorP = nt.getDoubleTopic("/elevator/P");
    DoubleTopic ElevatorI = nt.getDoubleTopic("/elevator/I");
    DoubleTopic ElevatorD = nt.getDoubleTopic("/elevator/D");
    double[] currentValues = new double[6];
    double[] previousValues = new double[6];
    DoublePublisher[] publishers = new DoublePublisher[6];
    DoubleSubscriber[] subscribers = new DoubleSubscriber[6];

    public PIDChanger() {
        publishers[0] = ArmP.publish();
        publishers[0].setDefault(Constants.ArmConstants.kp);
        publishers[1] = ArmI.publish();
        publishers[1].setDefault(Constants.ArmConstants.ki);
        publishers[2] = ArmD.publish();
        publishers[2].setDefault(Constants.ArmConstants.kd);
        publishers[3] = ElevatorP.publish();
        publishers[3].setDefault(Constants.ElevatorConstants.P_VAL);
        publishers[4] = ElevatorI.publish();
        publishers[4].setDefault(Constants.ElevatorConstants.I_VAL);
        publishers[5] = ElevatorD.publish();
        publishers[5].setDefault(Constants.ElevatorConstants.D_VAL);
        subscribers[0] = ArmP.subscribe(Constants.ArmConstants.kp);
        subscribers[1] = ArmI.subscribe(Constants.ArmConstants.ki);
        subscribers[2] = ArmD.subscribe(Constants.ArmConstants.kd);
        subscribers[3] = ElevatorP.subscribe(Constants.ElevatorConstants.P_VAL);
        subscribers[4] = ElevatorI.subscribe(Constants.ElevatorConstants.I_VAL);
        subscribers[5] = ElevatorD.subscribe(Constants.ElevatorConstants.D_VAL);
    }

    private void changeValues(double armP, double armI, double armD, double elevatorP, double elevatorI,
            double elevatorD) {
        {
            double PIDArray[] = { armP, armI, armD, elevatorP, elevatorI, elevatorD };
            for (int i = 0; i < previousValues.length; i++) {
                previousValues[i] = currentValues[i];
                // currentValues[i] = subscribers[i].get(previousValues[i]);
                if (previousValues[i] != PIDArray[i]) {
                    currentValues[i] = PIDArray[i];
                    previousValues[i] = PIDArray[i];
                    publishers[i].setDefault(previousValues[i]);
                }

            }
        }
    }

    public void periodic() {
        changeValues(1, 1, 1, 1, 1, 1);
        final long now = NetworkTablesJNI.now();
        Subsystem.arm.changePID(currentValues[0], currentValues[1], currentValues[2]);
        Subsystem.elevator.changePID(currentValues[3], currentValues[4], currentValues[5]);
        publishers[0].set(currentValues[0], now);
        publishers[1].set(currentValues[1], now);
        publishers[2].set(currentValues[2], now);
        publishers[3].set(currentValues[4], now);
        publishers[4].set(currentValues[5], now);
        publishers[5].set(currentValues[6], now);
        System.out.println("am I running");
    }

}
