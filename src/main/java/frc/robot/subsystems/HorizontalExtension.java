package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.HorizontalExtensionConstants;

public class HorizontalExtension extends SubsystemBase {
    private final SparkMax motor;

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
    
    private double currentPosition = 0.0;
    private double positionOffset = 0.0;
    
    public HorizontalExtension(){
        motor = new SparkMax(DeviceIDs.HORIZONTAL_EXTENSION, MotorType.kBrushless);

        motor.configure(HorizontalExtensionConstants.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor.getEncoder().setPosition(0.0);
        
        pid.setTolerance(ElevatorConstants.positionTolerance, ElevatorConstants.velocityTolerance);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    private double getRawPosition() {
        var position = motor.getEncoder().getPosition();
        return position;
    }

    public double getPosition() {
        return currentPosition;
    }

    public double getVelocity() {
        return motor.getEncoder().getVelocity();
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

        SmartDashboard.putNumber("horizontal extension pos", getPosition());
        SmartDashboard.putNumber("horizontal extension power", motor.get());
        SmartDashboard.putNumber("horizontal extension velocity", getVelocity());

    }
    
}