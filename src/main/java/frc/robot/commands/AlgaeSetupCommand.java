// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import java.util.function.BooleanSupplier;

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
public class AlgaeSetupCommand extends SequentialCommandGroup {
  /** Creates a new BlueRightTwoCoral. */

  SwerveDrivetrain drive;
  Elevator elevator;
  Arm arm;
  Manipulator manipulator;

  public AlgaeSetupCommand(SwerveDrivetrain drive, Elevator elevator, Arm arm, Manipulator manipulator, RobotCommands commands) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drive = drive;
    this.arm = arm;
    this.manipulator = manipulator;
    this.elevator = elevator;

    addRequirements(drive, elevator, arm, manipulator);

    addCommands(
        Commands.sequence(
            new DriveToReefAlgaeSetup(drive, elevator, arm, manipulator),
            Commands.either(commands.ballPickupL3FromHome(), commands.ballPickupL2FromHome(), isHighSupplier),
            new DriveToReefAlgaePickup(drive),
            commands.release()
        )
    );

  }

  BooleanSupplier isHighSupplier = () -> drive.getIsHigh();

}
