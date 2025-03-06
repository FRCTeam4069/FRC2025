// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
    private final SparkMax intake;
    private final SparkMax kicker;

    public Manipulator() {
        intake = new SparkMax(DeviceIDs.MANIPULATOR_INTAKE, MotorType.kBrushless);
        kicker = new SparkMax(DeviceIDs.MANIPULATOR_KICKER, MotorType.kBrushless);

        intake.configure(ManipulatorConstants.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kicker.configure(ManipulatorConstants.kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intake.getEncoder().setPosition(0.0);
        kicker.getEncoder().setPosition(0.0);

    }

    public void setIntake(double power) {
        intake.set(power);
    }

    public void setKicker(double power) {
        kicker.set(power);
    }

    public Command runIntake() {
        return runOnce(() -> setIntake(ManipulatorConstants.intakePower));
    }

    public Command stopIntake() {
        return runOnce(() -> setIntake(0.0));
    }

    public Command runKicker(DoubleSupplier power) {
        return run(() -> setKicker(power.getAsDouble()));
    }

    public Command defaultCommand(DoubleSupplier power) {
        return new Command() {
            @Override
            public void execute() {
                setKicker(power.getAsDouble());
                // if (Math.abs(power.getAsDouble()) > 0.05) {
                //     setIntake(ManipulatorConstants.intakeHoldPower);
                // }
            }
        };
    }

    public Command kickLeft() {
        return Commands.sequence(
            new InstantCommand(() -> setIntake(0.6)),
            new InstantCommand(() -> setKicker(-ManipulatorConstants.kickerPlacePower)), 
            Commands.waitSeconds(ManipulatorConstants.kickerPlaceTime), 
            new InstantCommand(() -> setIntake(0.0)),
            new InstantCommand(() -> setKicker(0.0)));
    }

    public Command kickRight() {
        return Commands.sequence(
            new InstantCommand(() -> setIntake(0.6)),
            new InstantCommand(() -> setKicker(ManipulatorConstants.kickerPlacePower)), 
            Commands.waitSeconds(ManipulatorConstants.kickerPlaceTime), 
            new InstantCommand(() -> setIntake(0.0)),
            new InstantCommand(() -> setKicker(0.0)));
    }

    @Override
    public void periodic() {
        
    }
}
