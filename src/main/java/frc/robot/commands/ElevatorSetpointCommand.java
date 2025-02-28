package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSetpointCommand extends Command {
    private final Elevator elevator;
    private final double targetPosition;

    public ElevatorSetpointCommand(Elevator elevator, double position) {
        this.elevator = elevator;
        this.targetPosition = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(targetPosition);
    }

    // @Override
    // public boolean isFinished() {
    //     double error = Math.abs(targetPosition - elevator.getPosition());
    //     return error < 2.0; // Consider done if within 2 units
    // }
}