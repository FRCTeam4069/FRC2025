// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.DrivetrainPIDController;

public class DriveToClimb extends Command {
    private final SwerveDrivetrain drive;
    private final DrivetrainPIDController controller;
    private Pose2d setpoint;
    private Alliance alliance;
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("target pose", Pose2d.struct).publish();
    private StructPublisher<Translation2d> vecPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("translation", Translation2d.struct).publish();

    public DriveToClimb(SwerveDrivetrain drive) {
        this.drive = drive;
        this.controller = new DrivetrainPIDController(DrivetrainConstants.pidToPositionConstants);

        addRequirements(drive);
    }

    public double getDistance(Pose2d currentPose, Pose2d closestPose) {
        var deltaX = currentPose.getX() - closestPose.getX();
        var deltaY = currentPose.getY() - closestPose.getY();
        return Math.hypot(deltaX, deltaY);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        } else {
            alliance = Alliance.Blue;
        }

        ArrayList<Pose2d> climbPoses = new ArrayList<>();

        if (alliance == Alliance.Blue) {
            for (Pose2d pose : DrivetrainConstants.blueClimbPoses) {
                climbPoses.add(pose);
            }
        } else {
            for (Pose2d pose : DrivetrainConstants.redClimbPoses) {
                climbPoses.add(pose);
            }
        }

        Pose2d currentPose = drive.getPose();
        Pose2d closestPose = climbPoses.get(0);
        double minimumDistance = Double.MAX_VALUE;

        for (Pose2d climbPose : climbPoses) {
            double distance = getDistance(currentPose, climbPose);
            if (distance < minimumDistance) {
                closestPose = climbPose;
                minimumDistance = distance;
            }
        }

        setpoint = closestPose;
        posePublisher.set(setpoint);

        controller.reset(drive.getPose(), ChassisSpeeds.fromRobotRelativeSpeeds(drive.getRobotRelativeSpeeds(), drive.getRotation2d()));
        controller.calculate(drive.getPose(), setpoint);
    }

    @Override
    public void execute() {
        drive.fieldOrientedDrive(controller.calculate(drive.getPose(), setpoint));
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
