package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX left;
    private final TalonFX right;

    private DutyCycleOut leftOutput = new DutyCycleOut(0.0);
    private DutyCycleOut rightOutput = new DutyCycleOut(0.0);

    private ProfiledPIDController pid = new ProfiledPIDController(
        ElevatorConstants.pidCoefficients.kP(), 
        ElevatorConstants.pidCoefficients.kP(), 
        ElevatorConstants.pidCoefficients.kP(), 
        ElevatorConstants.constraints);

    private ElevatorFeedforward ff = new ElevatorFeedforward(
        ElevatorConstants.ffCoefficients.kS(), 
        ElevatorConstants.ffCoefficients.kG(), 
        ElevatorConstants.ffCoefficients.kV(),
        ElevatorConstants.ffCoefficients.kA());

    private boolean lastAtBottom = false; 
    private boolean atBottom = false; 
    
    private double currentPosition = 0.0;
    private double positionOffset = 0.0;

    private final DigitalInput leftLimit;
    private final DigitalInput rightLimit;
    
    public Elevator(){
        left = new TalonFX(DeviceIDs.ELEVATOR_LEFT, "rio");
        right = new TalonFX(DeviceIDs.ELEVATOR_RIGHT, "rio");

        left.getConfigurator().apply(ElevatorConstants.leftConfig);
        right.getConfigurator().apply(ElevatorConstants.rightConfig);

        left.setPosition(0);
        right.setPosition(0);

        leftLimit = new DigitalInput(DeviceIDs.ELEVATOR_LIMIT_LEFT);
        rightLimit = new DigitalInput(DeviceIDs.ELEVATOR_LIMIT_RIGHT);

        pid.setTolerance(ElevatorConstants.positionTolerance, ElevatorConstants.velocityTolerance);
        
    }

    public void setPower(double power) {
        left.setControl(leftOutput.withOutput(power).withLimitReverseMotion(atBottom));
        right.setControl(rightOutput.withOutput(power).withLimitReverseMotion(atBottom));
    }

    private double getRawPosition() {
        var position = rotationsToMeters((left.getPosition().getValueAsDouble() + right.getPosition().getValueAsDouble()) / 2.0);
        return position;
    }

    public double getPosition() {
        return currentPosition;
    }

    public double rotationsToMeters(double rotations) {
        return rotations * 2 * Math.PI * ElevatorConstants.radius;
    }

    public void stop() {
        setPower(0);
    }

    public Command drive(DoubleSupplier power) {
        return run(() -> setPower(power.getAsDouble()));
    }

    @Override
    public void periodic() {
        currentPosition = getRawPosition() - positionOffset;

        atBottom = !leftLimit.get() || !rightLimit.get();

        if (atBottom && !lastAtBottom) {
            positionOffset += currentPosition;
        }

        lastAtBottom = atBottom;

        SmartDashboard.putNumber("elevator position", currentPosition);
        SmartDashboard.putNumber("left elevator position", rotationsToMeters(left.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("right elevator position", rotationsToMeters(right.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("elevator position", currentPosition);
        SmartDashboard.putBoolean("elevator limit", atBottom);
        SmartDashboard.putNumber("left elevator power", left.get());
        SmartDashboard.putNumber("right elevator power", right.get());
    }
    
}