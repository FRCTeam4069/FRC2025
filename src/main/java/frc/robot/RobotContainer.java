// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.subsystems.TestModule;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

// adding a comment for testing
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static final SwerveDrivetrain drive = new SwerveDrivetrain();
    // public static final TestModule test = new TestModule();
 
    private final CommandXboxController controller0 = new CommandXboxController(0);
    private final CommandXboxController controller1 = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drive.setDefaultCommand(
            new FieldCentricDrive(
                drive,
                () -> -controller0.getLeftY(),
                () -> -controller0.getLeftX(),
                () -> -controller0.getRightX(),
                () -> controller0.y().getAsBoolean(),
                () -> controller0.b().getAsBoolean()));
        
        autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser.addOption("driveQuasistaticForward", drive.alignForward().andThen(drive.driveSysIdRoutine.quasistatic(Direction.kForward)));
        // autoChooser.addOption("driveQuasistaticReverse", drive.alignForward().andThen(drive.driveSysIdRoutine.quasistatic(Direction.kReverse)));
        // autoChooser.addOption("driveDynamicForward", drive.alignForward().andThen(drive.driveSysIdRoutine.dynamic(Direction.kForward)));
        // autoChooser.addOption("driveDynamicReverse", drive.alignForward().andThen(drive.driveSysIdRoutine.dynamic(Direction.kReverse)));

        // autoChooser.addOption("steerQuasistaticForward", (drive.steerSysIdRoutine.quasistatic(Direction.kForward)));
        // autoChooser.addOption("steerQuasistaticReverse", (drive.steerSysIdRoutine.quasistatic(Direction.kReverse)));
        // autoChooser.addOption("steerDynamicForward", (drive.steerSysIdRoutine.dynamic(Direction.kForward)));
        // autoChooser.addOption("steerDynamicReverse", (drive.steerSysIdRoutine.dynamic(Direction.kReverse)));

        autoChooser.addOption("align", drive.alignForward());

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        controller0.a().onTrue(drive.resetHeadingCommand());
        controller0.start().onTrue(new InstantCommand(() -> drive.resetPose(new Pose2d())));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

