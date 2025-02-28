// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmPIDCommand extends Command {
    private final Arm arm;
    private final double pitch;
    private final double roll;

    public ArmPIDCommand(Arm arm, double pitch, double roll) {
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
        arm.setPosition(pitch, roll);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return arm.atPosition();
    }
}
