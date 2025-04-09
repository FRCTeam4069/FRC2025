// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.ColorSensorV3.MainControl;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToClimb;
import frc.robot.commands.DriveToReef;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.commands.PIDToPosition;
import frc.robot.commands.PIDToPositionClamped;
import frc.robot.commands.autos.BlueMiddleBall;
import frc.robot.commands.autos.BlueRightTwoCoral;
import frc.robot.commands.autos.PointFactory;
import frc.robot.commands.autos.VelocityPoint;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HorizontalExtension;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.TestModule;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Climber;
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
    public static final Arm arm = new Arm();
    public static final Elevator elevator = new Elevator();
    public static final Manipulator manipulator = new Manipulator();
    public static final Climber climber = new Climber();
    private final RobotCommands commands;
 
    private final CommandXboxController controller0 = new CommandXboxController(0);
    private final CommandXboxController controller1 = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        commands = new RobotCommands(drive, arm, elevator, manipulator, climber);
        registerAutoCommands();

        autoChooser = AutoBuilder.buildAutoChooser();

        // addSysIdCommands()

        autoChooser.addOption("blue right two coral", new BlueRightTwoCoral(drive, elevator, arm, manipulator, commands));
        autoChooser.addOption("blue middle ball", new BlueMiddleBall(drive, elevator, arm, manipulator, commands));
        autoChooser.addOption("test", new PIDToPositionClamped(drive, new Pose2d(5.0, 1.37, Rotation2d.fromDegrees(135.0)), PointFactory.createPointList(new VelocityPoint(1.0, 0.97), new VelocityPoint(0.15, 0.10), new VelocityPoint(0.1, 0.075))));

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
        // controller0.a().onTrue(drive.resetHeadingCommand());
        // controller0.start().onTrue(new InstantCommand(() -> drive.resetPose(new Pose2d())));

        controller0.leftBumper().onTrue(new DriveToReef(drive, true, () -> false)).onFalse(drive.stopOnceCommand());
        controller0.rightBumper().onTrue(new DriveToReef(drive, false, () -> false)).onFalse(drive.stopOnceCommand());
        controller0.leftTrigger().onTrue(new DriveToReef(drive, true, () -> true)).onFalse(drive.stopOnceCommand());
        controller0.rightTrigger().onTrue(new DriveToReef(drive, false, () -> true)).onFalse(drive.stopOnceCommand());
        controller0.a().onTrue(new DriveToClimb(drive)).onFalse(drive.stopOnceCommand());

        controller0.povLeft().onTrue(commands.ballRemoveL2());
        controller0.povRight().onTrue(commands.ballRemoveL3());

        // controller1.a().onTrue(arm.pid(ArmConstants.humanPlayer, 0));
        // controller1.b().onTrue(arm.pid(ArmConstants.rotatePoint, 0));
        // controller1.x().onTrue(arm.pid(ArmConstants.humanPlayer, ArmConstants.placeRoll));
        // controller1.y().onTrue(arm.pid(ArmConstants.rotatePoint, ArmConstants.placeRoll));

        // controller1.a().onTrue(elevator.pid(0.0));
        // controller1.b().onTrue(elevator.pid(0.3));
        // controller1.x().onTrue(elevator.pid(1.0));
        // controller1.y().onTrue(elevator.pid(1.3));

        controller1.a().onTrue(commands.home());
        controller1.b().onTrue(commands.humanPlayer());
        controller1.povLeft().onTrue(commands.L2());
        controller1.povRight().onTrue(commands.L3());
        controller1.povUp().onTrue(commands.L4());
        controller1.povDown().onTrue(commands.L1());
        controller1.rightBumper().onTrue(commands.place()).onFalse(commands.release());

        controller1.x().onTrue(commands.ballPickupL2());
        controller1.y().onTrue(commands.ballPickupL3());
        controller1.leftBumper().onTrue(commands.ballLaunch());
        controller1.start().onTrue(commands.ballPlace());
        controller1.back().onTrue(commands.humanPlayerOneCoral());

        // controller1.leftBumper().onTrue(climber.pid(ClimberConstants.forwardLimit));
        controller1.leftTrigger(0.8).onTrue(commands.climbStart());
        controller1.rightTrigger(0.8).onTrue(climber.winch());
        controller0.back().onTrue(commands.groundIntakeHorizontal());
     
        // controller1.rightTrigger(0.05).onTrue(manipulator.runIntake()).onFalse(manipulator.stopIntake());
        // controller1.leftTrigger(0.05).onTrue(manipulator.runIntake()).onFalse(manipulator.stopIntake());

        // controller1.y().onTrue(manipulator.runIntake());
        // controller1.x().onTrue(manipulator.stopIntake());
        // controller1.a().onTrue(manipulator.kickLeft());
        // controller1.b().onTrue(manipulator.kickRight());

    }

    private void registerAutoCommands() {
        NamedCommands.registerCommand("stop drivetrain", drive.stopCommand());
        NamedCommands.registerCommand("place l4", commands.placeL4());
        NamedCommands.registerCommand("l4", commands.L4Auto().andThen(Commands.waitSeconds(0.01)).andThen(commands.place().withTimeout(1.0)));
        NamedCommands.registerCommand("release", commands.release());
        NamedCommands.registerCommand("human player", commands.humanPlayer());
        NamedCommands.registerCommand("home", commands.home());
        NamedCommands.registerCommand("ground intake", commands.groundIntakeVertical());
        NamedCommands.registerCommand("drive bottom p1", new PIDToPosition(drive, new Pose2d(5.05, 2.69, Rotation2d.fromDegrees(120.0)), false)); //right
        NamedCommands.registerCommand("drive bottom p2", new PIDToPosition(drive, new Pose2d(3.61, 2.86, Rotation2d.fromDegrees(60.0)), false)); //left
        NamedCommands.registerCommand("drive bottom p3", new PIDToPosition(drive, new Pose2d(4.00, 2.87, Rotation2d.fromDegrees(60.0)), true));
        NamedCommands.registerCommand("blue left p1", new PIDToPosition(drive, new Pose2d(5.30, 5.02, Rotation2d.fromDegrees(-120.0)), true));
        NamedCommands.registerCommand("blue left p2", new PIDToPosition(drive, new Pose2d(3.72, 5.03, Rotation2d.fromDegrees(-60.0)), true));
        NamedCommands.registerCommand("red right drive p1", new PIDToPosition(drive, new Pose2d(12.59, 5.22, Rotation2d.fromDegrees(-60.0)), true));
        NamedCommands.registerCommand("red right drive p2", new PIDToPosition(drive, new Pose2d(13.86, 5.03, Rotation2d.fromDegrees(-120.0)), true));
        NamedCommands.registerCommand("red left drive p1", new PIDToPosition(drive, new Pose2d(12.57, 2.86, Rotation2d.fromDegrees(60)), true));
        NamedCommands.registerCommand("red left drive p2", new PIDToPosition(drive, new Pose2d(13.82, 3.03, Rotation2d.fromDegrees(120)), true));
        NamedCommands.registerCommand("blue right middle drive p1", new PIDToPosition(drive, new Pose2d(5.76, 3.83, Rotation2d.fromDegrees(180.0)), true));
        NamedCommands.registerCommand("red right middle drive p1", new PIDToPosition(drive, new Pose2d(11.78, 4.22, Rotation2d.fromDegrees(0)), true));
        NamedCommands.registerCommand("drive closest l4 right", new DriveToReef(drive, false, () -> true));
        NamedCommands.registerCommand("drive closest l4 left", new DriveToReef(drive, true, () -> true));
    }

    private void addSysIdCommands() {
        autoChooser.addOption("driveQuasistaticForward", (drive.driveSysIdRoutine.quasistatic(Direction.kForward)));
        autoChooser.addOption("driveQuasistaticReverse", (drive.driveSysIdRoutine.quasistatic(Direction.kReverse)));
        autoChooser.addOption("driveDynamicForward", (drive.driveSysIdRoutine.dynamic(Direction.kForward)));
        autoChooser.addOption("driveDynamicReverse", (drive.driveSysIdRoutine.dynamic(Direction.kReverse)));

        autoChooser.addOption("steerQuasistaticForward", (drive.steerSysIdRoutine.quasistatic(Direction.kForward)));
        autoChooser.addOption("steerQuasistaticReverse", (drive.steerSysIdRoutine.quasistatic(Direction.kReverse)));
        autoChooser.addOption("steerDynamicForward", (drive.steerSysIdRoutine.dynamic(Direction.kForward)));
        autoChooser.addOption("steerDynamicReverse", (drive.steerSysIdRoutine.dynamic(Direction.kReverse)));
        
    }

    public Command defaultDriveCommand() {
        return new FieldCentricDrive(
                drive,
                () -> -controller0.getLeftY(),
                () -> -controller0.getLeftX(),
                () -> -controller0.getRightX(),
                () -> controller0.getHID().getLeftBumperButton(),
                () -> false,
                // () -> controller0.getHID().getYButton(),
                () -> controller0.getHID().getXButton(),
                () -> controller0.getHID().getBButton(),
                () -> controller0.getHID().getRightBumperButton());
    }

    public Command defaultElevatorCommand() {
        return elevator.drive(() -> -controller1.getLeftY());
    }

    public Command defaultArmCommand() {
        var command = new InstantCommand();
        command.addRequirements(arm);
        return command;
    }

    public Command defaultManipulatorCommand() {
        var command = manipulator.defaultCommand(() -> (Math.pow(controller1.getRightTriggerAxis(), 3) - Math.pow(controller1.getLeftTriggerAxis(), 3)));
        command.addRequirements(manipulator);

        return command;
    }

    public Command defaultClimberCommand() {
        // return climber.defaultCommand(() -> MathUtil.applyDeadband(Math.pow(controller1.getHID().getRightTriggerAxis(), 3) + (controller0.getHID().getStartButton() ? -0.6 : 0.0), 0.1), (() -> arm.getState() == ArmState.HOME));
        return climber.drive(() -> controller1.getHID().getRightY());
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