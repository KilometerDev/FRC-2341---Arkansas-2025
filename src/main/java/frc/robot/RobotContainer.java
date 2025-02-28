// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.ElevatorSetpointCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Joystick joystick2 = new Joystick(1);
    private final int elevatorManualControlButton = 4;
    public final int elevatorManualControlaxis = 5;
    private final int L1button = 0;
    private final int L2button = 2;
    private final int L3button = 1;
    private final int L4button = 3;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator elevator = new Elevator();

    // public double joystickX = joystick.getLeftX();
    // public double joystickY = joystick.getLeftY();
    // public double joystickTwist = joystick.getRightX();

    // public double functionAppliedJoystickX = (1/0.81)*(joystickX-0.1)*(joystickX-0.1);
    // public double functionAppliedJoystickY = (1/0.81)*(joystickY-0.1)*(joystickY-0.1);
    // public double functionAppliedJoystickTwist = (1/0.81)*(joystickTwist-0.1)*(joystickTwist-0.1);

    // public double deadbandedJoystickMagnitudeX = MathUtil.applyDeadband(functionAppliedJoystickX, 0.1);
    // public double deadbandedJoystickMagnitudeY = MathUtil.applyDeadband(functionAppliedJoystickY, 0.1);
    // public double deadbandedJoystickMagnitudeTwist = MathUtil.applyDeadband(functionAppliedJoystickTwist, 0.1);

    // public double finalJoystickX = MathUtil.applyDeadband((1/0.81)*(joystick.getLeftX()-0.1)*(joystick.getLeftX()-0.1), 0.1);


    public RobotContainer() {
        configureBindings();
        elevator.setDefaultCommand(new ElevatorManualCommand(elevator, joystick2, elevatorManualControlaxis));
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive
                .withVelocityX(((joystick.getLeftX()) / Math.abs(joystick.getLeftX()))*(MathUtil.applyDeadband((1/0.81)*(joystick.getLeftX()-0.1)*(joystick.getLeftX()-0.1), 0.1) * MaxSpeed)) // Drive forward with negative Y (forward)
                .withVelocityY(((joystick.getLeftY()) / Math.abs(joystick.getLeftY()))*-MathUtil.applyDeadband((1/0.81)*(joystick.getLeftY()-0.1)*(joystick.getLeftY()-0.1), 0.1) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(((joystick.getRightX()) / Math.abs(joystick.getRightX()))*-MathUtil.applyDeadband((1/0.81)*(joystick.getRightX()-0.1)*(joystick.getRightX()-0.1), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


        // SETUP ELEVATOR PRESETS
        new JoystickButton(joystick2, L1button).onTrue(new ElevatorSetpointCommand(elevator, 0.0));  // TODO: Find L1
        new JoystickButton(joystick2, L2button).onTrue(new ElevatorSetpointCommand(elevator, 1.0));  // TODO: Find L2
        new JoystickButton(joystick2, L3button).onTrue(new ElevatorSetpointCommand(elevator, 2.0));  // TODO: Find L3
        new JoystickButton(joystick2, L4button).onTrue(new ElevatorSetpointCommand(elevator, 3.0));  // TODO: Find L4

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}