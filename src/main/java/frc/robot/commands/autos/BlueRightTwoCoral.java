// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PIDToPosition;
import frc.robot.constants.DrivetrainConstants;
import static frc.robot.constants.DrivetrainConstants.getReefPose;
import frc.robot.constants.DrivetrainConstants.ReefPoses;
import frc.robot.constants.DrivetrainConstants.HumanPlayerStations;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueRightTwoCoral extends SequentialCommandGroup {
  /** Creates a new BlueRightTwoCoral. */
  public BlueRightTwoCoral(SwerveDrivetrain drive, Elevator elevator, Arm arm, Manipulator manipulator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive, elevator, arm, manipulator);
    
    addCommands(
        new PIDToPosition(drive, getReefPose(HumanPlayerStations.BlueRight, ReefPoses.BottomRight), true)
        
    );
  }
}
