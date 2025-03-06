// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPlaceL1 extends SequentialCommandGroup {
    /** Creates a new ArmHome. */
    public ArmPlaceL1(Arm arm, Elevator elevator) {
        addCommands(
            new ParallelCommandGroup(
                elevator.pid(ElevatorConstants.l2),
                new ConditionalCommand(
                    arm.pid(40.0*(Math.PI/180), 90.0*(Math.PI/180)), 
                    new SequentialCommandGroup(
                        arm.pid(ArmConstants.rotatePoint, 0.0), 
                        arm.pid(ArmConstants.rotatePoint, 90.0*(Math.PI/180)),
                        arm.pid(35.0*(Math.PI/180), 90.0*(Math.PI/180))
                    ),
                    () -> arm.canRotate())
            ).handleInterrupt(() -> cancel())
        );
    }
}
