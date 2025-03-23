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

public class DriveToReef extends Command {
    private final SwerveDrivetrain drive;
    private final DrivetrainPIDController controller;
    private Pose2d setpoint;
    private Alliance alliance;
    private boolean left;
    private BooleanSupplier l4;
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("target pose", Pose2d.struct).publish();
    private StructPublisher<Translation2d> vecPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("translation", Translation2d.struct).publish();

    public DriveToReef(SwerveDrivetrain drive, boolean left, BooleanSupplier l4) {
        this.drive = drive;
        this.left = left;
        this.controller = new DrivetrainPIDController(DrivetrainConstants.pidToPositionConstants);
        this.l4 = l4;

        addRequirements(drive);
    }

    public double getDistance(Pose2d currentPose, Pose2d closestPose) {
        var deltaX = currentPose.getX() - closestPose.getX();
        var deltaY = currentPose.getY() - closestPose.getY();
        return Math.hypot(deltaX, deltaY);
    }

    public Pose2d backAway(Pose2d pose, double distance) {
        var angle = pose.getRotation();

        Translation2d vec = new Translation2d(Units.inchesToMeters(distance), 0.0);
        vec = vec.rotateBy(angle);

        vecPublisher.set(vec);

        return new Pose2d(pose.getX() + vec.getX(), pose.getY() + vec.getY(), pose.getRotation());
    }

    @Override
    public void initialize() {
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
        double minimumDistance = Double.MAX_VALUE;

        for (Pose2d reefPose : reefPoses) {
            double distance = getDistance(currentPose, reefPose);
            if (distance < minimumDistance) {
                closestPose = reefPose;
                minimumDistance = distance;
            }
        }

        if (!l4.getAsBoolean()) {
            setpoint = closestPose;
        } else {
            setpoint = backAway(closestPose, -7.5);
        }
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
