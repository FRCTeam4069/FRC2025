// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmDriveCommand extends Command {
    private final Arm arm;
    private final DoubleSupplier pitch;
    private final DoubleSupplier roll;
    private double lastPitch = 0.0;
    private double lastRoll = 0.0;
    private boolean wasPitching = false;
    private boolean wasRolling = false;
    
    private ProfiledPIDController pitchPID = new ProfiledPIDController(ArmConstants.pitchDrivePIDCoefficients.kP(), ArmConstants.pitchDrivePIDCoefficients.kI(), ArmConstants.pitchDrivePIDCoefficients.kD(), ArmConstants.pitchConstraints);
    private ProfiledPIDController rollPID = new ProfiledPIDController(ArmConstants.rollDrivePIDCoefficients.kP(), ArmConstants.rollDrivePIDCoefficients.kI(), ArmConstants.rollDrivePIDCoefficients.kD(), ArmConstants.rollConstraints);

    public ArmDriveCommand(Arm arm, DoubleSupplier pitch, DoubleSupplier roll) {
        this.arm = arm;
        this.pitch = pitch;
        this.roll = roll;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.resetControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var pitchOutput = pitch.getAsDouble();
        var rollOutput = roll.getAsDouble();

        /*
        if (Math.abs(pitchOutput) < 0.03) {
            if (!wasPitching) {
                pitchPID.reset(new State(arm.getPitch(), arm.getPitchVelocity()));
            }
            pitchOutput = pitchPID.calculate(arm.getPitch(), lastPitch);
        } else {
            lastPitch = arm.getPitch();
            wasPitching = true;
        }

        if (Math.abs(rollOutput) < 0.03) {
            if (!wasRolling) {
                rollPID.reset(new State(arm.getRoll(), arm.getRollVelocity()));
            }
            rollOutput = rollPID.calculate(arm.getRoll(), lastRoll);
        } else {
            lastRoll = arm.getRoll();
        }
        */

        arm.drive(pitchOutput, rollOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
