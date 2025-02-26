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
import frc.robot.subsystems.swerve.SwerveModule;

public class TestModule extends SubsystemBase {
    private SwerveModule module;

    /** Creates a new TestSubsystem. */
    public TestModule() {
        module = new SwerveModule(DrivetrainConstants.blConfig, DrivetrainConstants.blCoefficients);
    }

    public Command drive(DoubleSupplier speed) {
        return run(() -> module.setDesiredState(new SwerveModuleState(0.01, new Rotation2d(speed.getAsDouble()*2*Math.PI))));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("steer pos", module.getSteerPosition());
        SmartDashboard.putNumber("encoder pos", module.getEncoderRadians());
        SmartDashboard.putNumber("voltage", module.getSteerVoltage());
    }

}
