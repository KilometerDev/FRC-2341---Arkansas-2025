package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorManualCommand extends Command {
    private final Elevator elevator;
    private final Joystick joystick;
    private final int joystickAxis;

    public ElevatorManualCommand(Elevator elevator, Joystick joystick, int axis) {
        this.elevator = elevator;
        this.joystick = joystick;
        this.joystickAxis = axis;
        addRequirements(elevator);
    }

    public void excecute () {
        double speed = -joystick.getRawAxis(joystickAxis);
        double deadbandSpeed = MathUtil.applyDeadband(speed, 0.1);
        elevator.manualControl(deadbandSpeed * 1); // scale to prevent full throttle
    }

    // public void end(boolean interrupted) {
    //     elevator.manualControl(0); // stop the elevator
    // }

    // public boolean isFinished() {
    //     return false; // run forever until interrupted
    // }
}
