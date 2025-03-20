// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.constants.DrivetrainConstants.DrivetrainPIDConstants;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;
import frc.robot.constants.DrivetrainConstants.Tolerances;

/** 
 * Runs pid controller on x, y, and heading
 */
public class DrivetrainPIDController {
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController headingController;

    public DrivetrainPIDController(DrivetrainPIDConstants constants) {
        this(
            constants.translationCoefficients(),
            constants.translationConstraints(),
            constants.translationTolerances(),
            constants.rotationCoefficients(),
            constants.rotationConstraints(),
            constants.rotationTolerances());
    }

    public DrivetrainPIDController(
        PIDCoefficients translationCoefficients, 
        Constraints translationConstraints, 
        Tolerances translationTolerances,
        PIDCoefficients rotationCoefficients, 
        Constraints rotationConstraints,
        Tolerances rotationTolerances
    ) {
        xController = new ProfiledPIDController(translationCoefficients.kP(), translationCoefficients.kI(), translationCoefficients.kD(), translationConstraints);
        yController = new ProfiledPIDController(translationCoefficients.kP(), translationCoefficients.kI(), translationCoefficients.kD(), translationConstraints);
        headingController = new ProfiledPIDController(rotationCoefficients.kP(), rotationCoefficients.kI(), rotationCoefficients.kD(), rotationConstraints);

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(translationTolerances.position(), translationTolerances.velocity());
        yController.setTolerance(translationTolerances.position(), translationTolerances.velocity());
        headingController.setTolerance(rotationTolerances.position(), rotationTolerances.velocity());
    }

    /**
     * calculate controller outputs
     * @param currentPose
     * @param targetPose
     * @return desired field centric ChassisSpeeds
     */
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose) {
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double headingSpeed = headingController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        return new ChassisSpeeds(xSpeed, ySpeed, headingSpeed);
    }

    /**
     * reset controllers and set setpoints to current pose
     * @param currentPose
     * @param currentSpeeds field relative
     */
    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        xController.setGoal(currentPose.getX());
        yController.setGoal(currentPose.getY());
        headingController.setGoal(currentPose.getRotation().getRadians());

        xController.reset(new State(currentPose.getX(), currentSpeeds.vxMetersPerSecond));
        yController.reset(new State(currentPose.getY(), currentSpeeds.vyMetersPerSecond));
        headingController.reset(new State(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond));
    }

    /**
     * must run calculate first to set setpoint
     * @return if all controllers within tolerances.
     */
    public boolean atSetpoint() {
        return xController.atGoal() && yController.atGoal() && headingController.atGoal();
    }

}
