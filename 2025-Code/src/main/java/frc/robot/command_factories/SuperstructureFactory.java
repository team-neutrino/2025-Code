package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.util.Subsystem;

public class SuperstructureFactory {

    // Should lower arm to intaking position and then intake algae
    public static Command GroundIntakeAlgae() {
        return new ParallelCommandGroup(
                ElevatorFactory.AlgaeIntake(),
                Subsystem.claw.intakeGamePiece());
    }

    // Should raise arm to processor and then outake algae
    public static Command Processor() {
        return new ParallelCommandGroup(
                ElevatorFactory.AlgaeScorer(),
                Subsystem.claw.outakeGamePiece());
    }
}
