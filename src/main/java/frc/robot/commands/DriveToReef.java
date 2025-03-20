// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.DrivetrainPIDController;

public class DriveToReef extends Command {
    private final SwerveDrivetrain drive;
    private final boolean left;
    private final DrivetrainPIDController controller;
    private Pose2d setpoint;
    private Alliance alliance;

    public DriveToReef(SwerveDrivetrain drive, boolean left) {
        this.drive = drive;
        this.left = left;
        this.controller = new DrivetrainPIDController(DrivetrainConstants.pidToPositionConstants);

        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        } else {
            alliance = Alliance.Blue;
        }

        ArrayList<Pose2d> reefPoses = new ArrayList<>();

        if (alliance == Alliance.Blue) {
            if (left) {
                for (Pose2d pose : DrivetrainConstants.blueLeftReefPoses) {
                    reefPoses.add(pose);
                }
            } else {
                for (Pose2d pose : DrivetrainConstants.blueRightReefPoses) {
                    reefPoses.add(pose);
                }
            }
        } else {
            if (left) {
                for (Pose2d pose : DrivetrainConstants.redLeftReefPoses) {
                    reefPoses.add(pose);
                }
            } else {
                for (Pose2d pose : DrivetrainConstants.redRightReefPoses) {
                    reefPoses.add(pose);
                }
            }
        }

        Pose2d currentPose = drive.getPose();
        Pose2d closestPose = reefPoses.get(0);

        for (Pose2d reefPose : reefPoses) {
            
        }

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        controller.reset(drive.getPose(), ChassisSpeeds.fromRobotRelativeSpeeds(drive.getRobotRelativeSpeeds(), drive.getRotation2d()));
        controller.calculate(drive.getPose(), new Pose2d(4.5, 1.6, new Rotation2d()));
    }

    @Override
    public void execute() {
        drive.fieldOrientedDrive(controller.calculate(drive.getPose(), new Pose2d(4.5, 1.6, new Rotation2d())));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
