package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

// subsystems/intake/IntakeIOSim.java
public class IntakeIOSim {
    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        // Here, create the intake simulation with respect to the intake on your real
        // robot
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                // Specify the type of game pieces that the intake can collect
                "Algae",
                // Specify the drivetrain to which this intake is attached
                driveTrain,
                // Width of the intake
                Inches.of(20),
                // The extension length of the intake beyond the robot's frame (when activated)
                Inches.of(8),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.FRONT,
                // The intake can hold up to 1 note
                1);
    }

}
