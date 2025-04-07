// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Pair;
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

public class PIDToPositionClamped extends Command {
    private final SwerveDrivetrain drive;
    private final DrivetrainPIDController controller;
    private Pose2d setpoint;
    private boolean l4;
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("target pose", Pose2d.struct).publish();
    private StructPublisher<Translation2d> vecPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("translation", Translation2d.struct).publish();
    private ArrayList<Pair<Double, Double>> velocityTargets = new ArrayList<>();

    public PIDToPositionClamped(SwerveDrivetrain drive, Pose2d pose, ArrayList<Pair<Double, Double>> velocityTargets) {
        this(drive, pose, false, velocityTargets);
    }

    public PIDToPositionClamped(SwerveDrivetrain drive, Pose2d pose, boolean l4, ArrayList<Pair<Double, Double>> velocityTargets) {
        this.drive = drive;
        this.controller = new DrivetrainPIDController(DrivetrainConstants.autoPidToPositionConstants);
        this.setpoint = pose;
        this.l4 = l4;
        this.velocityTargets = velocityTargets;

        addRequirements(drive);
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
        if (l4) {
            setpoint = backAway(setpoint, -7.5);
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
