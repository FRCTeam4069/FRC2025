// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotCommands;
import frc.robot.commands.PIDToPosition;
import frc.robot.commands.PIDToPositionClamped;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ElevatorConstants;

import static frc.robot.constants.DrivetrainConstants.autoCloseEnoughConstants;
import static frc.robot.constants.DrivetrainConstants.getHumanPlayerPose;
import static frc.robot.constants.DrivetrainConstants.getReefPose;

import java.util.ArrayList;

import frc.robot.constants.DrivetrainConstants.ReefPoses;
import frc.robot.constants.DrivetrainConstants.Tolerances;
import frc.robot.constants.DrivetrainConstants.DrivetrainPIDConstants;
import frc.robot.constants.DrivetrainConstants.HumanPlayerStations;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueMiddleBall extends SequentialCommandGroup {
  /** Creates a new BlueRightTwoCoral. */
  public BlueMiddleBall(SwerveDrivetrain drive, Elevator elevator, Arm arm, Manipulator manipulator, RobotCommands commands) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive, elevator, arm, manipulator);
    
    Pose2d placeFirst = getReefPose(HumanPlayerStations.BlueRight, ReefPoses.MiddleRight);
    Pose2d ballFrontFirst = new Pose2d(6.20, 4.02, Rotation2d.fromDegrees(180.0));
    Pose2d ballPickupFirst = new Pose2d(5.91, 4.02, Rotation2d.fromDegrees(180.0));
    Pose2d ballPickupSecond = new Pose2d(5.167, 5.239, Rotation2d.fromDegrees(-120.0));
    Pose2d ballFrontSecond = new Pose2d(5.356, 5.525, Rotation2d.fromDegrees(-120.0));

    addCommands(
        new InstantCommand(() -> drive.resetDrivePose(drive.getPose())),
        Commands.parallel(
            new PIDToPositionClamped(drive, placeFirst, true, PointFactory.createPointList(new VelocityPoint(1.0, 1.0), new VelocityPoint(0.8, 0.7), new VelocityPoint(0.6, 0.4), new VelocityPoint(0.3, 0.25), new VelocityPoint(0.1, 0.20))),
            Commands.sequence(
                Commands.waitSeconds(0.010),
                arm.autoHome(),
                elevator.pid(ElevatorConstants.l4Auto),
                arm.setState(ArmState.L4),
                arm.pid(ArmConstants.L4Pitch, arm.getPlaceRoll()),
                commands.place().withTimeout(0.8)
            )
        ),
        Commands.parallel(
            commands.placingToBallPickupL2(),
            Commands.sequence(
                Commands.waitSeconds(0.2),
                new PIDToPositionClamped(drive, ballFrontFirst, false, PointFactory.createPointList(new VelocityPoint(1.0, 1.0), new VelocityPoint(0.8, 0.7), new VelocityPoint(0.6, 0.5), new VelocityPoint(0.3, 0.4), new VelocityPoint(0.1, 0.30)))
            )
        ),
        Commands.deadline(
            new PIDToPositionClamped(drive, ballPickupFirst, false, PointFactory.createPointList(new VelocityPoint(1.0, 1.0), new VelocityPoint(0.8, 0.7), new VelocityPoint(0.6, 0.6), new VelocityPoint(0.3, 0.55), new VelocityPoint(0.1, 0.70))).withTimeout(1.0),
            manipulator.runIntake()
        ),
        Commands.parallel(
            drive.followPathCommand("blue middle ball"),
            Commands.sequence(
                Commands.waitSeconds(0.3),
                commands.ballPlace()
            )
        ),
        commands.ballLaunch(),
        Commands.deadline(
            drive.followPathCommand("blue middle ball back"),
            Commands.sequence(
                Commands.waitSeconds(0.1),
                commands.home()
            )
        ),
        Commands.parallel(
            Commands.sequence(
                new PIDToPositionClamped(drive, ballFrontSecond, false, PointFactory.createPointList(new VelocityPoint(1.0, 1.0), new VelocityPoint(0.8, 0.7), new VelocityPoint(0.6, 0.5), new VelocityPoint(0.3, 0.5), new VelocityPoint(0.1, 0.40))),
                new PIDToPositionClamped(drive, ballPickupSecond, false, PointFactory.createPointList(new VelocityPoint(1.0, 1.0), new VelocityPoint(0.8, 0.7), new VelocityPoint(0.6, 0.5), new VelocityPoint(0.3, 0.5), new VelocityPoint(0.1, 0.40))).withTimeout(1.0)
            ),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                commands.ballPickupL3()
            )
        ),
        Commands.parallel(
            drive.followPathCommand("blue middle second ball"),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                commands.ballPlace()
            )
        ),
        commands.ballLaunch(),
        Commands.parallel(
            drive.followPathCommand("blue middle second ball back"),
            Commands.sequence(
                Commands.waitSeconds(0.1),
                commands.home()
            )
        )

    );

    }

}
