// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.commands.DriveToReef;
import frc.robot.commands.FieldCentricDrive;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HorizontalExtension;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.TestModule;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.Arm.ArmState;
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
    //public static final HorizontalExtension horizontalExtension = new HorizontalExtension();
    // public static final TestModule test = new TestModule();
 
    private final CommandXboxController controller0 = new CommandXboxController(0);
    private final CommandXboxController controller1 = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        registerAutoCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser.addOption("driveQuasistaticForward", (drive.driveSysIdRoutine.quasistatic(Direction.kForward)));
        // autoChooser.addOption("driveQuasistaticReverse", (drive.driveSysIdRoutine.quasistatic(Direction.kReverse)));
        // autoChooser.addOption("driveDynamicForward", (drive.driveSysIdRoutine.dynamic(Direction.kForward)));
        // autoChooser.addOption("driveDynamicReverse", (drive.driveSysIdRoutine.dynamic(Direction.kReverse)));

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

        // controller0.povLeft().onTrue(drive.increaseOffset(Rotation2d.fromDegrees(-1.0)));
        // controller0.povRight().onTrue(drive.increaseOffset(Rotation2d.fromDegrees(1.0)));

        controller0.povLeft().onTrue(new DriveToReef(drive, true));
        controller0.povRight().onTrue(new DriveToReef(drive, false));


        // controller1.a().onTrue(arm.pid(ArmConstants.humanPlayer, 0));
        // controller1.b().onTrue(arm.pid(ArmConstants.rotatePoint, 0));
        // controller1.x().onTrue(arm.pid(ArmConstants.humanPlayer, ArmConstants.placeRoll));
        // controller1.y().onTrue(arm.pid(ArmConstants.rotatePoint, ArmConstants.placeRoll));


        // controller1.a().onTrue(elevator.pid(0.0));
        // controller1.b().onTrue(elevator.pid(0.3));
        // controller1.y().onTrue(elevator.pid(1.0));


        controller1.b().onTrue(humanPlayer());
        controller1.a().onTrue(home());
        controller1.povLeft().onTrue(L2());
        controller1.povRight().onTrue(L3());
        controller1.povUp().onTrue(L4());
        controller1.leftBumper().onTrue(place());
        controller1.rightBumper().onTrue(place());

        // controller1.x().onTrue(ballPickupL2());
        // controller1.y().onTrue(ballPickupL3());
        // controller1.back().onTrue(ballLaunch());
        // controller1.start().onTrue(ballPlace());

        // controller1.rightTrigger(0.05).onTrue(manipulator.runIntake()).onFalse(manipulator.stopIntake());
        // controller1.leftTrigger(0.05).onTrue(manipulator.runIntake()).onFalse(manipulator.stopIntake());

        // controller1.y().onTrue(manipulator.runIntake());
        // controller1.x().onTrue(manipulator.stopIntake());
        // controller1.a().onTrue(manipulator.kickLeft());
        // controller1.b().onTrue(manipulator.kickRight());

    }

    private void registerAutoCommands() {
        NamedCommands.registerCommand("stop drivetrain", drive.stopCommand());
        NamedCommands.registerCommand("place l4 left", placeL4Left());
        NamedCommands.registerCommand("place l4 right", placeL4Right());
        NamedCommands.registerCommand("human player", humanPlayer());
        NamedCommands.registerCommand("stop intake", manipulator.stopIntake());
        NamedCommands.registerCommand("home", home());
    }

    private Command placeL4Left() {
        return Commands.sequence();
    }

    private Command placeL4Right() {
        return Commands.sequence();
    }

    public Command defaultDriveCommand() {
        return new FieldCentricDrive(
                drive,
                () -> -controller0.getLeftY(),
                () -> -controller0.getLeftX(),
                () -> -controller0.getRightX(),
                () -> controller0.leftBumper().getAsBoolean(),
                () -> controller0.y().getAsBoolean(),
                () -> controller0.x().getAsBoolean(),
                () -> controller0.b().getAsBoolean(),
                () -> controller0.rightBumper().getAsBoolean());
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

    private Command L2() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, placeFromPlace(ArmState.L2)),
                Map.entry(ArmState.L2, placeFromPlace(ArmState.L2)),
                Map.entry(ArmState.L3, placeFromPlace(ArmState.L2)),
                Map.entry(ArmState.L4, placeFromPlace(ArmState.L2)),
                Map.entry(ArmState.HOME, placeFromPlace(ArmState.L2)),
                Map.entry(ArmState.HP, placeFromHP(ArmState.L2)),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }


    private Command L3() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, placeFromPlace(ArmState.L3)),
                Map.entry(ArmState.L2, placeFromPlace(ArmState.L3)),
                Map.entry(ArmState.L3, placeFromPlace(ArmState.L3)),
                Map.entry(ArmState.L4, placeFromPlace(ArmState.L3)),
                Map.entry(ArmState.HOME, placeFromPlace(ArmState.L3)),
                Map.entry(ArmState.HP, placeFromHP(ArmState.L3)),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command L4() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, placeFromPlace(ArmState.L4)),
                Map.entry(ArmState.L2, placeFromPlace(ArmState.L4)),
                Map.entry(ArmState.L3, placeFromPlace(ArmState.L4)),
                Map.entry(ArmState.L4, placeFromPlace(ArmState.L4)),
                Map.entry(ArmState.HOME, placeFromPlace(ArmState.L4)),
                Map.entry(ArmState.HP, placeFromHP(ArmState.L4)),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command home() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, homeFromPlace()),
                Map.entry(ArmState.L2, homeFromPlace()),
                Map.entry(ArmState.L3, homeFromPlace()),
                Map.entry(ArmState.L4, homeFromPlace()),
                Map.entry(ArmState.HOME, homeFromPlace()),
                Map.entry(ArmState.HP, homeFromHP()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command humanPlayer() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, humanPlayerFromPlace()),
                Map.entry(ArmState.L2, humanPlayerFromPlace()),
                Map.entry(ArmState.L3, humanPlayerFromPlace()),
                Map.entry(ArmState.L4, humanPlayerFromPlace()),
                Map.entry(ArmState.HOME, humanPlayerFromPlace()),
                Map.entry(ArmState.HP, humanPlayerFromHumanPlayer()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command place() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, place(ArmState.L1)),
                Map.entry(ArmState.L2, place(ArmState.L2)),
                Map.entry(ArmState.L3, place(ArmState.L3)),
                Map.entry(ArmState.L4, place(ArmState.L4)),
                Map.entry(ArmState.HOME, new InstantCommand()),
                Map.entry(ArmState.HP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command ballPickupL2FromHome() {
        return Commands.parallel(
            manipulator.intakeUntilHolding(),
            arm.pid(70.0*(Math.PI/180), 0.0),
            elevator.pid(ElevatorConstants.ballL2),
            arm.setState(ArmState.BALL_L2_PICKUP)
        );
    }

    private Command ballPickupL3FromHome() {
        return Commands.parallel(
            manipulator.intakeUntilHolding(),
            arm.pid(70.0*(Math.PI/180), 0.0),
            elevator.pid(ElevatorConstants.ballL3),
            arm.setState(ArmState.BALL_L3_PICKUP)
        );
    }

    private Command ballPlaceFromHome() {
        return Commands.sequence(
            Commands.parallel(
                elevator.pid(ElevatorConstants.l4),
                arm.setState(ArmState.BALL_PLACE),
                arm.pid(ArmConstants.balancePoint, 0.0)
                //manipulator.runIntake()
            )
            //arm.pid(ArmConstants.balancePoint, 0.0)
        );
    }

    private Command ballLaunch() {
        return Commands.sequence(
            manipulator.shoot()
        );
    }

    /**
     * back-end method
     * @param initialState inital arm state, between L1-L4
     * @return
     */
    private Command place(ArmState initialState) {
        double armPitch = ArmConstants.rotatePoint;
        double armRoll = arm.getPlaceRoll();
        switch (initialState) {
            case L1:
                armPitch = ArmConstants.L1Pitch;
                break;
            case L2:
                armPitch = ArmConstants.L2Pitch;
                break;
            case L3:
                armPitch = ArmConstants.L3Pitch;
                break;
            case L4:
                armPitch = ArmConstants.L4Pitch;
                break;
            default:
                break;
        }

        return Commands.sequence(
                arm.pid(ArmConstants.placePitch, arm.getPlaceRoll()),
                Commands.race(
                    Commands.waitSeconds(0.5),
                    manipulator.outtakeUntilRelease()
                ),
                arm.pid(armPitch, armRoll)
            );
    }

    /**
     * 
     * @param targetState must be L1-L4
     * @return
     */
    private Command placeFromPlace(Arm.ArmState targetState) {
        assert (targetState == ArmState.L1 || targetState == ArmState.L2 || targetState == ArmState.L3 || targetState == ArmState.L4);
        double elevatorPosition = elevator.getPosition();
        double armPitch = ArmConstants.rotatePoint;
        double armRoll = arm.getPlaceRoll();
        switch (targetState) {
            case L1:
                elevatorPosition = ElevatorConstants.l1;
                armPitch = ArmConstants.L1Pitch;
                break;
            case L2:
                elevatorPosition = ElevatorConstants.l2;
                armPitch = ArmConstants.L2Pitch;
                break;
            case L3:
                elevatorPosition = ElevatorConstants.l3;
                armPitch = ArmConstants.L3Pitch;
                break;
            case L4:
                elevatorPosition = ElevatorConstants.l4;
                armPitch = ArmConstants.L4Pitch;
                break;
            default:
                break;
        }

        return Commands.sequence(
            arm.pid(armPitch, armRoll),
            arm.setState(targetState),
            elevator.pid(elevatorPosition)
        );
    }

    /**
     * 
     * @param targetState must be L1-L4
     * @return
     */
    private Command placeFromHP(Arm.ArmState targetState) {
        assert (targetState == ArmState.L1 || targetState == ArmState.L2 || targetState == ArmState.L3 || targetState == ArmState.L4);
        arm.placeLeft = manipulator.getPlaceDirection();
        double elevatorPosition = elevator.getPosition();
        double armPitch = ArmConstants.rotatePoint;
        double armRoll = arm.getPlaceRoll();
        switch (targetState) {
            case L1:
                elevatorPosition = ElevatorConstants.l1;
                armPitch = ArmConstants.L1Pitch;
                break;
            case L2:
                elevatorPosition = ElevatorConstants.l2;
                armPitch = ArmConstants.L2Pitch;
                break;
            case L3:
                elevatorPosition = ElevatorConstants.l3;
                armPitch = ArmConstants.L3Pitch;
                break;
            case L4:
                elevatorPosition = ElevatorConstants.l4;
                armPitch = ArmConstants.L4Pitch;
                break;
            default:
                break;
        }

        return Commands.sequence(
            arm.pid(ArmConstants.rotatePoint, armRoll),
            arm.setState(targetState),
            elevator.pid(elevatorPosition),
            arm.pid(armPitch, armRoll)
        );
    }

    private Command homeFromPlace() {
        return Commands.sequence(
                    arm.pid(ArmConstants.rotatePoint, arm.getPlaceRoll()), 
                    Commands.waitSeconds(0.001),
                    arm.setState(ArmState.HOME),
                    elevator.pidToBottom()
                );
    }

    private Command homeFromHP() {
        arm.placeLeft = manipulator.getPlaceDirection();
        return Commands.sequence(
                    elevator.pidToBottom(),
                    arm.setState(ArmState.HOME),
                    arm.pid(ArmConstants.rotatePoint, arm.getPlaceRoll())
                );
    }

    private Command humanPlayerFromPlace() {
        return Commands.sequence(
            Commands.parallel(
                elevator.pidToBottom(),
                arm.pid(ArmConstants.rotatePoint, arm.getPlaceRoll())
            ),
            Commands.parallel(
                arm.setState(ArmState.HP),
                manipulator.intakeUntilHolding(),
                arm.pid(ArmConstants.humanPlayer, 0)
            )
        );
    }

    private Command humanPlayerFromHumanPlayer() {
        return Commands.parallel(
            elevator.pidToBottom(),
            arm.pid(ArmConstants.humanPlayer, 0),
            manipulator.intakeUntilHolding(),
            arm.setState(ArmState.HP)
        );
    }

    private Command humanPlayerFromHome() {
        return humanPlayerFromPlace();
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


