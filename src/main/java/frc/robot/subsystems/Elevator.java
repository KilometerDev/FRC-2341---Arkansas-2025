package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    // PID constants for both motors
    private static final double kP = 10;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.0;

    // Motion Magic constants
    // motion magic gives us the ability to control a constant velocity throughout or movement to the setpoint, instead of just rushing there and potentially breaking something
    // it gives more precise control over the elevator movement throughout the closed loop cycle

    private static final double kCruiseVelocity = 50;
    private static final double kAcceleration = 100;

    private static final double kElevatorMin = -1000;
    private static final double kElevatorMax = 1000;

    public Elevator() {
        masterMotor = new TalonFX(50);
        followerMotor = new TalonFX(51);

        var motorConfig = new TalonFXConfiguration();

        // set PID gains
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        motorConfig.Slot0.kS = kS;

        // Configure Motion Magic
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = kAcceleration;

        // Setup soft limits for safety
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kElevatorMax;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kElevatorMin;

        // Setup current limits for safety
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40; // this is our breaker size, may need to adjust

        // apply configs
        masterMotor.getConfigurator().apply(motorConfig);
        followerMotor.getConfigurator().apply(motorConfig);

        // set follower motor to mimic main motors movement, but reversed
        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), true));
    }

    public void setElevatorPosition(double position) {
        position = Math.max(kElevatorMin, Math.min(kElevatorMax, position));
        masterMotor.setControl(new MotionMagicVoltage(position));
    }

    public void manualControl(double speed) {
        masterMotor.setControl(new DutyCycleOut(speed));
    }

    public double getposition() {
        return masterMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic () {
        // System.out.println("Elevator Position: " + getposition());
    }
}