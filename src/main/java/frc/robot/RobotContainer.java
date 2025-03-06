// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
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

        controller1.b().onTrue(humanPlayer());
        // controller1.y().onTrue(manipulator.stopIntake());
        // controller1.x().onTrue(manipulator.kickLeft());
        // controller1.b().onTrue(manipulator.kickRight());
        controller1.a().onTrue(home());
        controller1.povLeft().onTrue(midL2());
        controller1.povRight().onTrue(midL3());
        controller1.povUp().onTrue(midL4());
        controller1.leftBumper().onTrue(placeLeft());
        controller1.rightBumper().onTrue(placeRight());
        controller1.x().onTrue(ballPickupL2());
        controller1.y().onTrue(ballPickupL3());
        controller1.back().onTrue(ballLaunch());
        controller1.start().onTrue(ballPlace());

        controller1.rightTrigger(0.05).onTrue(manipulator.runIntake()).onFalse(manipulator.stopIntake());
        controller1.leftTrigger(0.05).onTrue(manipulator.runIntake()).onFalse(manipulator.stopIntake());

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
        return Commands.sequence(midL4FromHome().withTimeout(7.0), placeLeft().withTimeout(2.0), Commands.waitSeconds(0.2), manipulator.kickLeft(), midPlace(ArmState.L4_PLACE));
    }

    private Command placeL4Right() {
        return Commands.sequence(midL4FromHome().withTimeout(7.0), placeRight().withTimeout(2.0), Commands.waitSeconds(0.2), manipulator.kickRight(), midPlace(ArmState.L4_PLACE));
    }

    public Command defaultDriveCommand() {
        return new FieldCentricDrive(
                drive,
                () -> -controller0.getLeftY(),
                () -> -controller0.getLeftX(),
                () -> -controller0.getRightX(),
                () -> controller0.y().getAsBoolean(),
                () -> controller0.b().getAsBoolean());
    }

    public Command defaultElevatorCommand() {
        return elevator.drive(() -> -controller1.getLeftY());
    }

    public Command defaultArmCommand() {
        return new InstantCommand(() -> {}, arm);
    }

    public Command defaultManipulatorCommand() {
        var command = manipulator.defaultCommand(() -> (Math.pow(controller1.getRightTriggerAxis(), 3) - Math.pow(controller1.getLeftTriggerAxis(), 3)));
        command.addRequirements(manipulator);

        return command;
    }

    private Command midL2() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, midL2FromHome()),
                Map.entry(ArmState.L2_MID, new InstantCommand()),
                Map.entry(ArmState.L3_MID, midL2FromHome()),
                Map.entry(ArmState.L4_MID, midL2FromHome()),
                Map.entry(ArmState.L1_PLACE, midPlace(ArmState.L1_PLACE)),
                Map.entry(ArmState.L2_PLACE, midPlace(ArmState.L2_PLACE)),
                Map.entry(ArmState.L3_PLACE, midPlace(ArmState.L3_PLACE)),
                Map.entry(ArmState.L4_PLACE, midPlace(ArmState.L4_PLACE)),
                Map.entry(ArmState.HOME, midL2FromHome()),
                Map.entry(ArmState.HP, midL2FromHome()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command midL3() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, midL3FromHome()),
                Map.entry(ArmState.L2_MID, midL3FromHome()),
                Map.entry(ArmState.L3_MID, new InstantCommand()),
                Map.entry(ArmState.L4_MID, midL3FromHome()),
                Map.entry(ArmState.L1_PLACE, midPlace(ArmState.L1_PLACE)),
                Map.entry(ArmState.L2_PLACE, midPlace(ArmState.L2_PLACE)),
                Map.entry(ArmState.L3_PLACE, midPlace(ArmState.L3_PLACE)),
                Map.entry(ArmState.L4_PLACE, midPlace(ArmState.L4_PLACE)),
                Map.entry(ArmState.HOME, midL3FromHome()),
                Map.entry(ArmState.HP, midL3FromHome()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command midL4() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, midL4FromHome()),
                Map.entry(ArmState.L2_MID, midL4FromHome()),
                Map.entry(ArmState.L3_MID, midL4FromHome()),
                Map.entry(ArmState.L4_MID, new InstantCommand()),
                Map.entry(ArmState.L1_PLACE, midPlace(ArmState.L1_PLACE)),
                Map.entry(ArmState.L2_PLACE, midPlace(ArmState.L2_PLACE)),
                Map.entry(ArmState.L3_PLACE, midPlace(ArmState.L3_PLACE)),
                Map.entry(ArmState.L4_PLACE, midPlace(ArmState.L4_PLACE)),
                Map.entry(ArmState.HOME, midL4FromHome()),
                Map.entry(ArmState.HP, midL4FromHome()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command home() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, homeFromMid()),
                Map.entry(ArmState.L2_MID, homeFromMid()),
                Map.entry(ArmState.L3_MID, homeFromMid()),
                Map.entry(ArmState.L4_MID, homeFromL4Mid()),
                Map.entry(ArmState.L1_PLACE, homeFromPlace()),
                Map.entry(ArmState.L2_PLACE, homeFromPlace()),
                Map.entry(ArmState.L3_PLACE, homeFromPlace()),
                Map.entry(ArmState.L4_PLACE, homeFromL4Place()),
                Map.entry(ArmState.HOME, new InstantCommand()),
                Map.entry(ArmState.HP, homeFromHP()),
                Map.entry(ArmState.BALL_L2_PICKUP, homeFromMid()),
                Map.entry(ArmState.BALL_L3_PICKUP, homeFromMid()),
                Map.entry(ArmState.BALL_PLACE, homeFromL4Mid())
            ), () -> arm.getState());
    }

    private Command humanPlayer() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, humanPlayerFromMid()),
                Map.entry(ArmState.L2_MID, humanPlayerFromMid()),
                Map.entry(ArmState.L3_MID, humanPlayerFromMid()),
                Map.entry(ArmState.L4_MID, humanPlayerFromL4Mid()),
                Map.entry(ArmState.L1_PLACE, humanPlayerFromPlace()),
                Map.entry(ArmState.L2_PLACE, humanPlayerFromPlace()),
                Map.entry(ArmState.L3_PLACE, humanPlayerFromPlace()),
                Map.entry(ArmState.L4_PLACE, humanPlayerFromL4Place()),
                Map.entry(ArmState.HOME, humanPlayerFromHome()),
                Map.entry(ArmState.HP, humanPlayerFromHome()),
                Map.entry(ArmState.BALL_L2_PICKUP, humanPlayerFromMid()),
                Map.entry(ArmState.BALL_L3_PICKUP, humanPlayerFromMid()),
                Map.entry(ArmState.BALL_PLACE, humanPlayerFromL4Mid())
            ), () -> arm.getState());
    }

    private Command placeLeft() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, leftPlace(ArmState.L1_MID)),
                Map.entry(ArmState.L2_MID, leftPlace(ArmState.L2_MID)),
                Map.entry(ArmState.L3_MID, leftPlace(ArmState.L3_MID)),
                Map.entry(ArmState.L4_MID, leftPlaceL4()),
                Map.entry(ArmState.L1_PLACE, midPlace(ArmState.L1_PLACE)),
                Map.entry(ArmState.L2_PLACE, midPlace(ArmState.L2_PLACE)),
                Map.entry(ArmState.L3_PLACE, midPlace(ArmState.L3_PLACE)),
                Map.entry(ArmState.L4_PLACE, midPlace(ArmState.L4_PLACE)),
                Map.entry(ArmState.HOME, new InstantCommand()),
                Map.entry(ArmState.HP, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command placeRight() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, rightPlace(ArmState.L1_MID)),
                Map.entry(ArmState.L2_MID, rightPlace(ArmState.L2_MID)),
                Map.entry(ArmState.L3_MID, rightPlace(ArmState.L3_MID)),
                Map.entry(ArmState.L4_MID, rightPlaceL4()),
                Map.entry(ArmState.L1_PLACE, midPlace(ArmState.L1_PLACE)),
                Map.entry(ArmState.L2_PLACE, midPlace(ArmState.L2_PLACE)),
                Map.entry(ArmState.L3_PLACE, midPlace(ArmState.L3_PLACE)),
                Map.entry(ArmState.L4_PLACE, midPlace(ArmState.L4_PLACE)),
                Map.entry(ArmState.HOME, new InstantCommand()),
                Map.entry(ArmState.HP, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command ballPickupL2() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, ballPickupL2FromEverywhere()),
                Map.entry(ArmState.L2_MID, ballPickupL2FromEverywhere()),
                Map.entry(ArmState.L3_MID, ballPickupL2FromEverywhere()),
                Map.entry(ArmState.L4_MID, ballPickupL2FromEverywhere()),
                Map.entry(ArmState.L1_PLACE, new InstantCommand()),
                Map.entry(ArmState.L2_PLACE, new InstantCommand()),
                Map.entry(ArmState.L3_PLACE, new InstantCommand()),
                Map.entry(ArmState.L4_PLACE, new InstantCommand()),
                Map.entry(ArmState.HOME, ballPickupL2FromEverywhere()),
                Map.entry(ArmState.HP, ballPickupL2FromEverywhere()),
                Map.entry(ArmState.BALL_L2_PICKUP, manipulator.runIntake()),
                Map.entry(ArmState.BALL_L3_PICKUP, ballPickupL2FromEverywhere()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command ballPickupL3() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, ballPickupL3FromEverywhere()),
                Map.entry(ArmState.L2_MID, ballPickupL3FromEverywhere()),
                Map.entry(ArmState.L3_MID, ballPickupL3FromEverywhere()),
                Map.entry(ArmState.L4_MID, ballPickupL3FromEverywhere()),
                Map.entry(ArmState.L1_PLACE, new InstantCommand()),
                Map.entry(ArmState.L2_PLACE, new InstantCommand()),
                Map.entry(ArmState.L3_PLACE, new InstantCommand()),
                Map.entry(ArmState.L4_PLACE, new InstantCommand()),
                Map.entry(ArmState.HOME, ballPickupL3FromEverywhere()),
                Map.entry(ArmState.HP, ballPickupL3FromEverywhere()),
                Map.entry(ArmState.BALL_L2_PICKUP, ballPickupL3FromEverywhere()),
                Map.entry(ArmState.BALL_L3_PICKUP, manipulator.runIntake()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand())
            ), () -> arm.getState());
    }

    private Command ballPlace() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1_MID, ballPlaceFromHome()),
                Map.entry(ArmState.L2_MID, ballPlaceFromHome()),
                Map.entry(ArmState.L3_MID, ballPlaceFromHome()),
                Map.entry(ArmState.L4_MID, ballPlaceFromHome()),
                Map.entry(ArmState.L1_PLACE, new InstantCommand()),
                Map.entry(ArmState.L2_PLACE, new InstantCommand()),
                Map.entry(ArmState.L3_PLACE, new InstantCommand()),
                Map.entry(ArmState.L4_PLACE, new InstantCommand()),
                Map.entry(ArmState.HOME, ballPlaceFromHome()),
                Map.entry(ArmState.HP, ballPlaceFromHome()),
                Map.entry(ArmState.BALL_L2_PICKUP, ballPlaceFromHome()),
                Map.entry(ArmState.BALL_L3_PICKUP, ballPlaceFromHome()),
                Map.entry(ArmState.BALL_PLACE, ballPlaceFromHome())
            ), () -> arm.getState());
    }

    private Command ballPickupL2FromEverywhere() {
        return Commands.parallel(
            manipulator.runIntake(),
            arm.pid(70.0*(Math.PI/180), 0.0),
            elevator.pid(ElevatorConstants.ballL2),
            arm.setState(ArmState.BALL_L2_PICKUP)
        );
    }

    private Command ballPickupL3FromEverywhere() {
        return Commands.parallel(
            manipulator.runIntake(),
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
                arm.pid(0.0, 0.0),
                manipulator.runIntake()
            ),
            arm.pid(ArmConstants.balancePoint, 0.0)
        );
    }

    private Command ballLaunch() {
        return Commands.sequence(
            manipulator.shoot()
        );
    }

    private Command leftPlace(ArmState initialState) {
        return Commands.parallel(
                Commands.sequence(
                    arm.setState(Arm.toPlace(initialState)),
                    arm.pidRoll(80.0*(Math.PI/180)),
                    new WaitCommand(0.2),
                    arm.pid(35.0*(Math.PI/180), 83.0*(Math.PI/180)),
                    manipulator.kickLeft()
                )
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private Command rightPlace(ArmState initialState) {
        return Commands.parallel(
                Commands.sequence(
                    arm.setState(Arm.toPlace(initialState)),
                    // arm.pid(ArmConstants.rotatePoint, -83.0*(Math.PI/180)),
                    arm.pidRoll(-80.0*(Math.PI/180)),
                    new WaitCommand(0.2),
                    arm.pid(35.0*(Math.PI/180), -83.0*(Math.PI/180)),
                    manipulator.kickRight()
                )
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private Command leftPlaceL4() {
        return Commands.parallel(
                Commands.sequence(
                    arm.setState(ArmState.L4_PLACE),
                    arm.pidRoll(80.0*(Math.PI/180)),
                    new WaitCommand(0.2),
                    arm.pid(ArmConstants.L4Pitch, 83.0*(Math.PI/180))
                    //manipulator.kickLeft()
                )
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private Command rightPlaceL4() {
        return Commands.parallel(
                Commands.sequence(
                    arm.setState(ArmState.L4_PLACE),
                    arm.pidRoll(-80.0*(Math.PI/180)),
                    new WaitCommand(0.2),
                    arm.pid(ArmConstants.L4Pitch, -83.0*(Math.PI/180))
                    //manipulator.kickRight()
                )
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private Command midPlace(ArmState initialState) {
        return Commands.sequence(
            arm.pid(60.0*(Math.PI/180), 0.0*(Math.PI/180))
        ).alongWith(arm.setState(Arm.toMid(initialState)));
    }

    private Command midL2FromHome() {
        return Commands.parallel(
                elevator.pid(ElevatorConstants.l2),
                Commands.sequence(
                    // manipulator.runIntake(),
                    arm.setState(ArmState.L2_MID),
                    arm.pid(ArmConstants.rotatePoint, 0.0),
                    manipulator.stopIntake()
                )
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);

    }

    private Command midL3FromHome() {
        return Commands.parallel(
                elevator.pid(ElevatorConstants.l3),
                Commands.sequence(
                    // manipulator.runIntake(),
                    arm.setState(ArmState.L3_MID),
                    arm.pid(ArmConstants.rotatePoint, 0.0),
                    manipulator.stopIntake()
                )
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private Command midL4FromHome() {
        return Commands.sequence(
            Commands.parallel(
                elevator.pid(ElevatorConstants.l4),
                arm.setState(ArmState.L4_MID),
                arm.pid(0.0, 0.0)
            ),
            arm.pid(ArmConstants.rotatePoint, 0.0),
            manipulator.stopIntake()
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private Command homeFromPlace() {
        return Commands.sequence(
                    // manipulator.runIntake(),
                    arm.pid(60.0*(Math.PI/180), 0.0), 
                    Commands.waitSeconds(0.001),
                    arm.pid(-90.0*(Math.PI/180), 0.0*(Math.PI/180)),
                    arm.setState(ArmState.HOME),
                    elevator.pid(0.0).alongWith(manipulator.stopIntake())
                );
    }

    private Command homeFromMid() {
        return Commands.sequence(
                    // manipulator.runIntake(),
                    arm.pid(-90.0*(Math.PI/180), 0.0*(Math.PI/180)),
                    arm.setState(ArmState.HOME),
                    manipulator.stopIntake()
                )
                .andThen(elevator.pid(0.0));
    }
    
    private Command homeFromL4Mid() {
        return Commands.sequence(
                    // manipulator.runIntake(),
                    Commands.deadline(
                        elevator.pid(0.0),
                        arm.setState(ArmState.HOME),
                        arm.pid(0.0*(Math.PI/180), 0.0*(Math.PI/180))
                    ),
                    arm.pid(-90.0*(Math.PI/180), 0.0*(Math.PI/180)),
                    manipulator.stopIntake()
                );
    }

    private Command homeFromL4Place() {
        return Commands.sequence(
                    // manipulator.runIntake(),
                    arm.pid(60.0*(Math.PI/180), 0.0), 
                    Commands.waitSeconds(0.001),
                    Commands.deadline(
                        elevator.pid(0.0),
                        arm.setState(ArmState.HOME),
                        arm.pid(0.0*(Math.PI/180), 0.0*(Math.PI/180))
                    ),
                    arm.pid(-90.0*(Math.PI/180), 0.0*(Math.PI/180)),
                    manipulator.stopIntake()
                );
    }

    private Command homeFromHP() {
        return Commands.parallel(
                    // manipulator.runIntake(),
                    arm.pid(-90.0*(Math.PI/180), 0.0*(Math.PI/180)),
                    elevator.pid(0.0),
                    arm.setState(ArmState.HOME)
                ).andThen(manipulator.stopIntake());
    }

    private Command humanPlayerFromPlace() {
        return Commands.parallel(
            manipulator.runIntake(),
            Commands.sequence(
                arm.pid(60.0*(Math.PI/180), 0.0), 
                arm.pid(ArmConstants.humanPlayer, 0.0)
            ),
            elevator.pid(ElevatorConstants.hp),
            arm.setState(ArmState.HP)
        );
    }

    private Command humanPlayerFromL4Place() {
        return Commands.sequence(
                    manipulator.runIntake(),
                    arm.pid(60.0*(Math.PI/180), 0.0), 
                    Commands.waitSeconds(0.001),
                    Commands.deadline(
                        elevator.pid(ElevatorConstants.hp),
                        arm.setState(ArmState.HP),
                        arm.pid(0.0, 0.0)
                    ),
                    arm.pid(ArmConstants.humanPlayer, 0.0)
                );
    }

    private Command humanPlayerFromL4Mid() {
        return Commands.sequence(
                    manipulator.runIntake(),
                    Commands.deadline(
                        elevator.pid(ElevatorConstants.hp),
                        arm.setState(ArmState.HP),
                        arm.pid(0.0, 0.0)
                    ),
                    arm.pid(ArmConstants.humanPlayer, 0.0)
                );
    }

    private Command humanPlayerFromMid() {
        return Commands.parallel(
            manipulator.runIntake(),
            arm.pid(ArmConstants.humanPlayer, 0.0),
            elevator.pid(ElevatorConstants.hp),
            arm.setState(ArmState.HP)
        );
    }

    private Command humanPlayerFromHome() {
        return Commands.parallel(
            manipulator.runIntake(),
            arm.pid(ArmConstants.humanPlayer, 0.0),
            elevator.pid(ElevatorConstants.hp),
            arm.setState(ArmState.HP)
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


