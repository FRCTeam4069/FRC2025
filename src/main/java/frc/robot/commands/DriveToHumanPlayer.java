// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class DriveToHumanPlayer extends Command {
    private final SwerveDrivetrain drive;
    private final DoubleSupplier joystick;

    private enum HumanPlayerStation {
        blueLeft,
        blueRight,
        redLeft,
        redRight
    }

    private HumanPlayerStation hp;

    public DriveToHumanPlayer(SwerveDrivetrain drive, DoubleSupplier joystick) {
        this.drive = drive;
        this.joystick = joystick;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        var pose = drive.getPose();

        if (pose.getX() < 8.8) {
            if (pose.getY() < 4.0) {
                hp = HumanPlayerStation.blueRight;
            } else {
                hp = HumanPlayerStation.blueLeft;
            }
        } else {
            if (pose.getY() < 4.0) {
                hp = HumanPlayerStation.redLeft;
            } else {
                hp = HumanPlayerStation.redRight;
            }
        }

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
