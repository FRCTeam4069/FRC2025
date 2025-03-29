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
        // controller0.a().onTrue(drive.resetHeadingCommand());
        // controller0.start().onTrue(new InstantCommand(() -> drive.resetPose(new Pose2d())));

        // controller0.povLeft().onTrue(drive.increaseOffset(Rotation2d.fromDegrees(-1.0)));
        // controller0.povRight().onTrue(drive.increaseOffset(Rotation2d.fromDegrees(1.0)));

        controller0.povLeft().onTrue(new DriveToReef(drive, true, () -> controller0.getHID().getYButton())).onFalse(drive.stopOnceCommand());
        controller0.povRight().onTrue(new DriveToReef(drive, false, () -> controller0.getHID().getYButton())).onFalse(drive.stopOnceCommand());
        controller0.a().onTrue(new DriveToClimb(drive)).onFalse(drive.stopOnceCommand());


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
        controller1.povDown().onTrue(L1());
        controller1.rightBumper().onTrue(place()).onFalse(release());

        controller1.x().onTrue(ballPickupL2());
        controller1.y().onTrue(ballPickupL3());
        controller1.leftBumper().onTrue(ballLaunch());
        controller1.start().onTrue(ballPlace());
        controller1.back().onTrue(humanPlayerOneCoral());

        // controller1.leftBumper().onTrue(climber.pid(ClimberConstants.forwardLimit));
        controller1.leftTrigger(0.8).onTrue(climbStart());
        controller0.back().onTrue(groundIntakeHorizontal());
     
        // controller1.rightTrigger(0.05).onTrue(manipulator.runIntake()).onFalse(manipulator.stopIntake());
        // controller1.leftTrigger(0.05).onTrue(manipulator.runIntake()).onFalse(manipulator.stopIntake());

        // controller1.y().onTrue(manipulator.runIntake());
        // controller1.x().onTrue(manipulator.stopIntake());
        // controller1.a().onTrue(manipulator.kickLeft());
        // controller1.b().onTrue(manipulator.kickRight());

    }

    private void registerAutoCommands() {
        NamedCommands.registerCommand("stop drivetrain", drive.stopCommand());
        NamedCommands.registerCommand("place l4", placeL4());
        NamedCommands.registerCommand("l4", L4().andThen(place().withTimeout(0.5)));
        NamedCommands.registerCommand("release", release());
        NamedCommands.registerCommand("human player", humanPlayer());
        NamedCommands.registerCommand("home", home());
        NamedCommands.registerCommand("ground intake", groundIntakeVertical());
        NamedCommands.registerCommand("drive bottom p1", new PIDToPosition(drive, new Pose2d(5.05, 2.69, Rotation2d.fromDegrees(120.0)), false)); //right
        NamedCommands.registerCommand("drive bottom p2", new PIDToPosition(drive, new Pose2d(3.61, 2.86, Rotation2d.fromDegrees(60.0)), false)); //left
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

    private Command placeL4() {
        return Commands.sequence(
            L4(),
            place().withTimeout(1.0),
            release()
        );
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
        return climber.defaultCommand(() -> MathUtil.applyDeadband(Math.pow(controller1.getHID().getRightTriggerAxis(), 3) + (controller0.getHID().getStartButton() ? -0.6 : 0.0), 0.1), (() -> arm.getState() == ArmState.HOME));
    }

    private Command L1() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, placeFromPlace(ArmState.L1)),
                Map.entry(ArmState.L2, placeFromPlace(ArmState.L1)),
                Map.entry(ArmState.L3, placeFromPlace(ArmState.L1)),
                Map.entry(ArmState.L4, placeFromPlace(ArmState.L1)),
                Map.entry(ArmState.HOME, placeFromPlace(ArmState.L1)),
                Map.entry(ArmState.HP, placeFromHP(ArmState.L1)),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, placeFromPlace(ArmState.L1))
            ), () -> arm.getState());
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
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, placeFromPlace(ArmState.L2))
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
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, placeFromPlace(ArmState.L3))
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
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, placeFromPlace(ArmState.L4))
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
                Map.entry(ArmState.BALL_L2_PICKUP, homeFromBallPickup()),
                Map.entry(ArmState.BALL_L3_PICKUP, homeFromBallPickup()),
                Map.entry(ArmState.BALL_PLACE, homeFromPlace()),
                Map.entry(ArmState.INTAKE_GROUND, homeFromPlace())
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
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, humanPlayerFromPlace())
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
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command release() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, release(ArmState.L1)),
                Map.entry(ArmState.L2, release(ArmState.L2)),
                Map.entry(ArmState.L3, release(ArmState.L3)),
                Map.entry(ArmState.L4, release(ArmState.L4)),
                Map.entry(ArmState.HOME, new InstantCommand()),
                Map.entry(ArmState.HP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState()).andThen(home());
    }

    private Command ballPickupL2() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, new InstantCommand()),
                Map.entry(ArmState.L2, new InstantCommand()),
                Map.entry(ArmState.L3, new InstantCommand()),
                Map.entry(ArmState.L4, new InstantCommand()),
                Map.entry(ArmState.HOME, ballPickupL2FromHome()),
                Map.entry(ArmState.HP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_PICKUP, ballPickupL2FromHome()),
                Map.entry(ArmState.BALL_L3_PICKUP, ballPickupL2FromHome()),
                Map.entry(ArmState.BALL_PLACE, ballPickupL2FromHome()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command ballPickupL3() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, new InstantCommand()),
                Map.entry(ArmState.L2, new InstantCommand()),
                Map.entry(ArmState.L3, new InstantCommand()),
                Map.entry(ArmState.L4, new InstantCommand()),
                Map.entry(ArmState.HOME, ballPickupL3FromHome()),
                Map.entry(ArmState.HP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_PICKUP, ballPickupL3FromHome()),
                Map.entry(ArmState.BALL_L3_PICKUP, ballPickupL3FromHome()),
                Map.entry(ArmState.BALL_PLACE, ballPickupL3FromHome()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }
    
    private Command ballPlace() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, new InstantCommand()),
                Map.entry(ArmState.L2, new InstantCommand()),
                Map.entry(ArmState.L3, new InstantCommand()),
                Map.entry(ArmState.L4, new InstantCommand()),
                Map.entry(ArmState.HOME, ballPlaceFromHome()),
                Map.entry(ArmState.HP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_PICKUP, ballPlaceFromHome()),
                Map.entry(ArmState.BALL_L3_PICKUP, ballPlaceFromHome()),
                Map.entry(ArmState.BALL_PLACE, ballPlaceFromHome()),
                Map.entry(ArmState.INTAKE_GROUND, ballPlaceFromHome())
            ), () -> arm.getState());
    }

    private Command climbStart() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, new InstantCommand()),
                Map.entry(ArmState.L2, new InstantCommand()),
                Map.entry(ArmState.L3, new InstantCommand()),
                Map.entry(ArmState.L4, new InstantCommand()),
                Map.entry(ArmState.HOME, climber.filp().withTimeout(0.75).andThen(climber.stopPivotCommand())),
                Map.entry(ArmState.HP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command humanPlayerOneCoral() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, new InstantCommand()),
                Map.entry(ArmState.L2, new InstantCommand()),
                Map.entry(ArmState.L3, new InstantCommand()),
                Map.entry(ArmState.L4, new InstantCommand()),
                Map.entry(ArmState.HOME, new InstantCommand()),
                Map.entry(ArmState.HP, humanPlayerDown()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command groundIntakeVertical() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, groundIntakeVerticalFromHome()),
                Map.entry(ArmState.L2, groundIntakeVerticalFromHome()),
                Map.entry(ArmState.L3, groundIntakeVerticalFromHome()),
                Map.entry(ArmState.L4, groundIntakeVerticalFromHome()),
                Map.entry(ArmState.HOME, groundIntakeVerticalFromHome()),
                Map.entry(ArmState.HP, groundIntakeVerticalFromHumanPlayer()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command groundIntakeHorizontal() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, groundIntakeHorizontalFromHome()),
                Map.entry(ArmState.L2, groundIntakeHorizontalFromHome()),
                Map.entry(ArmState.L3, groundIntakeHorizontalFromHome()),
                Map.entry(ArmState.L4, groundIntakeHorizontalFromHome()),
                Map.entry(ArmState.HOME, groundIntakeHorizontalFromHome()),
                Map.entry(ArmState.HP, groundIntakeHorizontalFromHumanPlayer()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command ballPickupL2FromHome() {
        return Commands.parallel(
            manipulator.runIntake(),
            arm.pid(ArmConstants.ballPitch, arm.getPlaceRoll()),
            elevator.pid(ElevatorConstants.ballL2),
            arm.setState(ArmState.BALL_L2_PICKUP)
        );
    }

    private Command ballPickupL3FromHome() {
        return Commands.parallel(
            manipulator.runIntake(),
            arm.pid(ArmConstants.ballPitch, arm.getPlaceRoll()),
            elevator.pid(ElevatorConstants.ballL3),
            arm.setState(ArmState.BALL_L3_PICKUP)
        );
    }

    private Command ballPlaceFromHome() {
        return Commands.sequence(
            Commands.parallel(
                elevator.pid(ElevatorConstants.ballPlace),
                arm.setState(ArmState.BALL_PLACE),
                arm.pid(ArmConstants.ballStowPitch, arm.getPlaceRoll())
                //manipulator.runIntake()
            ),
            arm.pid(ArmConstants.ballPlacePitch, arm.getPlaceRoll())
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
        double armPlacePitch = ArmConstants.placePitch;
        switch (initialState) {
            case L1:
                armPitch = ArmConstants.L1Pitch;
                armPlacePitch = ArmConstants.L1Pitch;
                armRoll = 0.0;
                break;
            case L2:
                armPitch = ArmConstants.L2Pitch;
                break;
            case L3:
                armPitch = ArmConstants.L3Pitch;
                break;
            case L4:
                armPitch = ArmConstants.L4Pitch;
                armPlacePitch = ArmConstants.l4PlacePitch;
                break;
            default:
                break;
        }

        return Commands.sequence(
                    //elevator.pid(Elevator.armStateToPosition(initialState)),
                arm.pid(armPlacePitch, armRoll)
                    
            );
    }

    /**
     * back-end method
     * @param initialState inital arm state, between L1-L4
     * @return
     */
    private Command release(ArmState initialState) {
        double armPitch = ArmConstants.rotatePoint;
        double armReturnPitch = ArmConstants.l2ReturnPitch;
        double armRoll = arm.getPlaceRoll();
        double outtakeSpeed = ManipulatorConstants.outtakePower;
        switch (initialState) {
            case L1:
                armPitch = ArmConstants.L1Pitch;
                armReturnPitch = ArmConstants.L1Pitch;
                armRoll = 0.0;
                outtakeSpeed = -0.05;
                break;
            case L2:
                armPitch = ArmConstants.L2Pitch;
                break;
            case L3:
                armPitch = ArmConstants.L3Pitch;
                break;
            case L4:
                // armPitch = ArmConstants.L4Pitch;
                armReturnPitch = ArmConstants.l4ReturnPitch;

                return Commands.sequence(
                    Commands.parallel(
                        manipulator.setIntakeOnce(outtakeSpeed),
                        elevator.pid(Elevator.armStateToDownPosition(initialState)),
                        Commands.sequence(
                            Commands.waitSeconds(0.200),
                            arm.pid(armReturnPitch, armRoll)
                        )
                    ),
                    Commands.parallel(
                        arm.setState(ArmState.HOME),
                        manipulator.outtakeUntilRelease(outtakeSpeed),
                        elevator.pidToBottom(),
                        arm.pid(ArmConstants.rotatePoint, armRoll)
                    )

                    );
            default:
                break;
        }

        return Commands.sequence(
            Commands.parallel(
                manipulator.setIntakeOnce(outtakeSpeed),
                elevator.pid(Elevator.armStateToDownPosition(initialState)),
                arm.pid(armReturnPitch, armRoll)
            ),
            Commands.parallel(
                arm.setState(ArmState.HOME),
                manipulator.outtakeUntilRelease(outtakeSpeed),
                elevator.pidToBottom(),
                arm.pid(ArmConstants.rotatePoint, armRoll)
            )
            
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
                armRoll = 0.0;
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
            elevator.pid(elevatorPosition),
            arm.setState(targetState),
            arm.pid(armPitch, armRoll)
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
            manipulator.stopIntake(),
            elevator.pidToBottom(),
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

    private Command homeFromBallPickup() {
        return Commands.sequence(
                    arm.pid(ArmConstants.ballStowPitch, arm.getPlaceRoll()), 
                    Commands.waitSeconds(0.001),
                    arm.setState(ArmState.HOME),
                    elevator.pidToBottom()
                );
    }

    private Command homeFromHP() {
        arm.placeLeft = manipulator.getPlaceDirection();
        return Commands.sequence(
                    manipulator.stopIntake(),
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
            arm.setState(ArmState.HP),
            Commands.race(
                Commands.waitUntil(() -> arm.getPitch() < -10.0),
                arm.pid(ArmConstants.preHumanPlayer, 0.0)
            ),
            Commands.parallel(
                Commands.sequence(
                    Commands.parallel(
                        arm.pid(ArmConstants.preHumanPlayer, 0.0),
                        elevator.pid(ElevatorConstants.hp)
                    ),
                    arm.pid(ArmConstants.humanPlayer, 0)
                ),
                manipulator.intakeUntilHolding()
            )
        );
    }

    private Command humanPlayerFromHumanPlayer() {
        return Commands.parallel(
            elevator.pid(ElevatorConstants.hp),
            arm.pid(ArmConstants.humanPlayer, 0),
            manipulator.intakeUntilHolding(),
            arm.setState(ArmState.HP)
        );
    }

    private Command humanPlayerFromHome() {
        return humanPlayerFromPlace();
    }

    private Command humanPlayerDown() {
        return Commands.parallel(
            elevator.pid(ElevatorConstants.hpOneCoral),
            arm.pid(ArmConstants.humanPlayerOneCoral, 0),
            manipulator.intakeUntilHolding(),
            arm.setState(ArmState.HP)
        );
    }

    private Command groundIntakeVerticalFromHome() {
        return Commands.parallel(
            elevator.pid(ElevatorConstants.groundIntake),
            arm.pid(ArmConstants.groundIntakeVerticalPitch, arm.getPlaceRoll()),
            manipulator.intakeUntilHolding(),
            arm.setState(ArmState.INTAKE_GROUND)
        );
    }

    private Command groundIntakeVerticalFromHumanPlayer() {
        return Commands.sequence(
            elevator.pid(ElevatorConstants.groundIntake),
            Commands.parallel(
                arm.pid(ArmConstants.groundIntakeVerticalPitch, arm.getPlaceRoll()),
                manipulator.intakeUntilHolding(),
                arm.setState(ArmState.INTAKE_GROUND)
            )
        );
    }

    private Command groundIntakeHorizontalFromHome() {
        return Commands.parallel(
            elevator.pid(ElevatorConstants.groundIntake),
            arm.pid(ArmConstants.groundIntakeHorizontalPitch, Units.degreesToRadians(45.0)),
            manipulator.intakeUntilHolding(),
            arm.setState(ArmState.INTAKE_GROUND)
        );
    }

    private Command groundIntakeHorizontalFromHumanPlayer() {
        return Commands.sequence(
            elevator.pid(ElevatorConstants.groundIntake),
            Commands.parallel(
                arm.pid(ArmConstants.groundIntakeHorizontalPitch, Units.degreesToRadians(45.0)),
                manipulator.intakeUntilHolding(),
                arm.setState(ArmState.INTAKE_GROUND)
            )
        );
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


