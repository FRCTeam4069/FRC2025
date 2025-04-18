// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.constants.DrivetrainConstants.blueAlgaePickupPosition;
import static frc.robot.constants.DrivetrainConstants.redAlgaePickupPosition;

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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.DrivetrainPIDController;

public class DriveToReefAlgaeSetup extends Command {
    private final SwerveDrivetrain drive;
    public final Elevator elevator;
    public final Arm arm;
    public final Manipulator manipulator;
    private final DrivetrainPIDController controller;
    private Pose2d setpoint;
    private Alliance alliance;
    private boolean isHigh;
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("target pose", Pose2d.struct).publish();
    private StructPublisher<Translation2d> vecPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("translation", Translation2d.struct).publish();

    public DriveToReefAlgaeSetup(SwerveDrivetrain drive, Elevator elevator, Arm arm, Manipulator manipulator) {
        this.drive = drive;
        this.elevator = elevator;
        this.arm = arm;
        this.manipulator = manipulator;
        this.controller = new DrivetrainPIDController(DrivetrainConstants.pidToPositionConstants);

        addRequirements(drive, elevator, arm, manipulator);
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

        ArrayList<Pose2d> reefPoses = new ArrayList<>();

        if (alliance == Alliance.Blue) {
            for (Pose2d pose : DrivetrainConstants.blueAlgaeSetupPosition) {
                reefPoses.add(pose);
            }
        } else {
            for (Pose2d pose : DrivetrainConstants.redAlgaeSetupPosition) {
                reefPoses.add(pose);
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

        setpoint = closestPose;
        posePublisher.set(setpoint);

        isHigh = false;
        for (int i = 0; i<redAlgaePickupPosition.length; i+=2){
            if (setpoint == redAlgaePickupPosition[i] || setpoint == blueAlgaePickupPosition[i]) {
                isHigh = true;
            }
        }
        controller.reset(drive.getPose(), ChassisSpeeds.fromRobotRelativeSpeeds(drive.getRobotRelativeSpeeds(), drive.getRotation2d()));
        controller.calculate(drive.getPose(), setpoint);

        drive.setIsHigh(isHigh);
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
