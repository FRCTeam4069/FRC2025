// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.DrivetrainConstants;

public class TestSubsystem extends SubsystemBase {
    private SparkFlex steer;
    private CANcoder encoder;
    private PIDController steerPID;
    private SimpleMotorFeedforward driveFF;

    /** Creates a new TestSubsystem. */
    public TestSubsystem() {
        var moduleCoefficients = DrivetrainConstants.blCoefficients;

        steerPID = new PIDController(moduleCoefficients.steerKP(), moduleCoefficients.steerKI(), moduleCoefficients.steerKD());
        steer = new SparkFlex(DeviceIDs.STEER_BL, MotorType.kBrushless);
        driveFF = new SimpleMotorFeedforward(moduleCoefficients.driveKS(), moduleCoefficients.driveKV(), moduleCoefficients.driveKA());

        SparkFlexConfig steerConfig = new SparkFlexConfig();
        steerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DrivetrainConstants.steerCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0)
            .apply(new EncoderConfig().positionConversionFactor(DrivetrainConstants.steerConversionFactor)
                .velocityConversionFactor(DrivetrainConstants.steerConversionFactor * 1.0/60.0)); // deg/min to deg/sec

        steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new CANcoder(DeviceIDs.ENCODER_BL, "rio");

        MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
        encoderConfig.SensorDirection = true ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
        encoderConfig.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetOffset = DrivetrainConstants.blEncoderOffset;
        encoder.getConfigurator().apply(encoderConfig);

        steerPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setPower(double power) {
        steer.set(power);
    }

    public double getPosition() {
        // read encoder
        return steer.getEncoder().getPosition();
    }

    public void stop() {
        steer.stopMotor();
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(encoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = getRotation2d();
        
        desiredState.optimize(currentRotation);

        double steerOutput = steerPID.calculate(currentRotation.getRadians(), desiredState.angle.getRadians());
        steer.set(steerOutput);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("steer pos", steer.getEncoder().getPosition());
        SmartDashboard.putNumber("encoder pos", encoder.getPosition().getValueAsDouble());
    }

    public Command drive(DoubleSupplier power) {
        return run(() -> setPower(power.getAsDouble()));
    }
}
