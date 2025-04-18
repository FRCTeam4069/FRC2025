// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
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
public class BlueLeftTwoCoral extends SequentialCommandGroup {
  /** Creates a new BlueRightTwoCoral. */
  public BlueLeftTwoCoral(SwerveDrivetrain drive, Elevator elevator, Arm arm, Manipulator manipulator, RobotCommands commands) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive, elevator, arm, manipulator);
    
    Pose2d placeFirst = getReefPose(HumanPlayerStations.BlueLeft, ReefPoses.TopRight);//
    Pose2d humanPickup = getHumanPlayerPose(HumanPlayerStations.BlueLeft);//
    Pose2d placeSecond = getReefPose(HumanPlayerStations.BlueLeft, ReefPoses.TopLeft);//
    Pose2d placeThird = getReefPose(HumanPlayerStations.BlueRight, ReefPoses.TopLeft);//

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
        Commands.deadline(
            commands.placingToHumanPlayer(),
            Commands.sequence(
                Commands.waitSeconds(0.2),
                new PIDToPositionClamped(drive, humanPickup, PointFactory.createPointList(new VelocityPoint(1.7, 1.0), new VelocityPoint(1.2, 0.70), new VelocityPoint(1.1, 0.15), new VelocityPoint(0.5, 0.10), new VelocityPoint(0.17, 0.07)))
            )
        ),
        Commands.parallel(
            new PIDToPositionClamped(drive, placeSecond, true, PointFactory.createPointList(new VelocityPoint(1.5, 1.0), new VelocityPoint(1.0, 0.70), new VelocityPoint(0.8, 0.6), new VelocityPoint(0.25, 0.20))),
            Commands.sequence(
                Commands.waitSeconds(0.1),
                commands.L4Auto(),
                commands.place().withTimeout(0.8)
            )
        ),
        Commands.deadline(
            Commands.sequence(
                commands.placingToHumanPlayer()
            ),
            Commands.sequence(
                Commands.waitSeconds(0.2),
                new PIDToPositionClamped(drive, humanPickup, PointFactory.createPointList(new VelocityPoint(1.6, 1.0), new VelocityPoint(1.0, 0.70), new VelocityPoint(0.9, 0.15), new VelocityPoint(0.3, 0.10), new VelocityPoint(0.17, 0.07)))
            )
        ),
        Commands.parallel(
            new PIDToPositionClamped(drive, placeThird, true, PointFactory.createPointList(new VelocityPoint(1.5, 1.0), new VelocityPoint(1.0, 0.70), new VelocityPoint(0.8, 0.5), new VelocityPoint(0.55, 0.3), new VelocityPoint(0.1, 0.20))),
            Commands.sequence(
                Commands.waitSeconds(0.1),
                commands.L4Auto(),
                commands.place().withTimeout(0.8)
            )
        ),
        Commands.sequence(
            commands.release()
        )
    );
  }

}
