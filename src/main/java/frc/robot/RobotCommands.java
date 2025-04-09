package frc.robot;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ManipulatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class RobotCommands {
    private final SwerveDrivetrain drive;
    private final Arm arm;
    private final Elevator elevator;
    private final Manipulator manipulator;
    private final Climber climber;

    public RobotCommands(SwerveDrivetrain drive, Arm arm, Elevator elevator, Manipulator manipulator, Climber climber) {
        this.drive = drive;
        this.arm = arm;
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.climber = climber;
    }

    public Command placeL4() {
        return Commands.sequence(
            L4Auto(),
            place().withTimeout(1.0),
            release()
        );
    }


    public Command L1() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, placeFromPlace(ArmState.L1)),
                Map.entry(ArmState.BALL_L3_REMOVE, placeFromPlace(ArmState.L1)),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, placeFromPlace(ArmState.L1))
            ), () -> arm.getState());
    }

    public Command L2() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, placeFromPlace(ArmState.L2)),
                Map.entry(ArmState.BALL_L3_REMOVE, placeFromPlace(ArmState.L2)),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, placeFromPlace(ArmState.L2))
            ), () -> arm.getState());
    }


    public Command L3() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, placeFromPlace(ArmState.L3)),
                Map.entry(ArmState.BALL_L3_REMOVE, placeFromPlace(ArmState.L3)),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, placeFromPlace(ArmState.L3))
            ), () -> arm.getState());
    }

    public Command L4() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, placeFromPlace(ArmState.L4)),
                Map.entry(ArmState.BALL_L3_REMOVE, placeFromPlace(ArmState.L4)),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, placeFromPlace(ArmState.L4))
            ), () -> arm.getState());
    }

    public Command L4Auto() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, placeFromPlaceAuto(ArmState.L4)),
                Map.entry(ArmState.L2, placeFromPlaceAuto(ArmState.L4)),
                Map.entry(ArmState.L3, placeFromPlaceAuto(ArmState.L4)),
                Map.entry(ArmState.L4, placeFromPlaceAuto(ArmState.L4)),
                Map.entry(ArmState.HOME, placeFromPlaceAuto(ArmState.L4)),
                Map.entry(ArmState.HP, placeFromHPAuto(ArmState.L4)),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_REMOVE, placeFromPlaceAuto(ArmState.L4)),
                Map.entry(ArmState.BALL_L3_REMOVE, placeFromPlaceAuto(ArmState.L4)),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, placeFromPlaceAuto(ArmState.L4))
            ), () -> arm.getState());
    }

    public Command home() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, homeFromPlace()),
                Map.entry(ArmState.BALL_L3_REMOVE, homeFromPlace()),
                Map.entry(ArmState.BALL_PLACE, homeFromPlace()),
                Map.entry(ArmState.INTAKE_GROUND, homeFromPlace())
            ), () -> arm.getState());
    }

    public Command humanPlayer() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, humanPlayerFromPlace()),
                Map.entry(ArmState.BALL_L3_REMOVE, humanPlayerFromPlace()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, humanPlayerFromPlace())
            ), () -> arm.getState());
    }

    public Command place() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    public Command release() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState()).andThen(home());
    }

    public Command ballPickupL2() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, ballPickupL2FromHome()),
                Map.entry(ArmState.BALL_L3_REMOVE, ballPickupL2FromHome()),
                Map.entry(ArmState.BALL_PLACE, ballPickupL2FromHome()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    public Command ballPickupL3() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, ballPickupL3FromHome()),
                Map.entry(ArmState.BALL_L3_REMOVE, ballPickupL3FromHome()),
                Map.entry(ArmState.BALL_PLACE, ballPickupL3FromHome()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }
    
    public Command ballPlace() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, ballPlaceFromHome()),
                Map.entry(ArmState.INTAKE_GROUND, ballPlaceFromHome())
            ), () -> arm.getState());
    }

    public Command climbStart() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, new InstantCommand()),
                Map.entry(ArmState.L2, new InstantCommand()),
                Map.entry(ArmState.L3, new InstantCommand()),
                Map.entry(ArmState.L4, new InstantCommand()),
                Map.entry(ArmState.HOME, climber.filp().withTimeout(3.0).andThen(climber.stopPivotCommand())),
                Map.entry(ArmState.HP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_PICKUP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    public Command humanPlayerOneCoral() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_L3_REMOVE, new InstantCommand()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    public Command groundIntakeVertical() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, groundIntakeVerticalFromHome()),
                Map.entry(ArmState.BALL_L3_REMOVE, groundIntakeVerticalFromHome()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    public Command groundIntakeHorizontal() {
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
                Map.entry(ArmState.BALL_L2_REMOVE, groundIntakeHorizontalFromHome()),
                Map.entry(ArmState.BALL_L3_REMOVE, groundIntakeHorizontalFromHome()),
                Map.entry(ArmState.BALL_PLACE, new InstantCommand()),
                Map.entry(ArmState.INTAKE_GROUND, new InstantCommand())
            ), () -> arm.getState());
    }

    public Command ballRemoveL2() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, ballRemoveL2FromHome()),
                Map.entry(ArmState.L2, ballRemoveL2FromHome()),
                Map.entry(ArmState.L3, ballRemoveL2FromHome()),
                Map.entry(ArmState.L4, ballRemoveL2FromHome()),
                Map.entry(ArmState.HOME, ballRemoveL2FromHome()),
                Map.entry(ArmState.HP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_PICKUP, ballRemoveL2FromHome()),
                Map.entry(ArmState.BALL_L3_PICKUP, ballRemoveL2FromHome()),
                Map.entry(ArmState.BALL_L2_REMOVE, ballRemoveL2FromHome()),
                Map.entry(ArmState.BALL_L3_REMOVE, ballRemoveL2FromHome()),
                Map.entry(ArmState.BALL_PLACE, ballRemoveL2FromHome()),
                Map.entry(ArmState.INTAKE_GROUND, ballRemoveL2FromHome())
            ), () -> arm.getState());
    }

    public Command ballRemoveL3() {
        return Commands.select(
            Map.ofEntries(
                Map.entry(ArmState.L1, ballRemoveL3FromHome()),
                Map.entry(ArmState.L2, ballRemoveL3FromHome()),
                Map.entry(ArmState.L3, ballRemoveL3FromHome()),
                Map.entry(ArmState.L4, ballRemoveL3FromHome()),
                Map.entry(ArmState.HOME, ballRemoveL3FromHome()),
                Map.entry(ArmState.HP, new InstantCommand()),
                Map.entry(ArmState.BALL_L2_PICKUP, ballRemoveL3FromHome()),
                Map.entry(ArmState.BALL_L3_PICKUP, ballRemoveL3FromHome()),
                Map.entry(ArmState.BALL_L2_REMOVE, ballRemoveL3FromHome()),
                Map.entry(ArmState.BALL_L3_REMOVE, ballRemoveL3FromHome()),
                Map.entry(ArmState.BALL_PLACE, ballRemoveL3FromHome()),
                Map.entry(ArmState.INTAKE_GROUND, ballRemoveL3FromHome())
            ), () -> arm.getState());
    }

    public Command ballPickupL2FromHome() {
        return Commands.parallel(
            manipulator.runIntake(),
            arm.pid(ArmConstants.ballPitch, arm.getPlaceRoll()),
            elevator.pid(ElevatorConstants.ballL2),
            arm.setState(ArmState.BALL_L2_PICKUP)
        );
    }

    public Command ballPickupL3FromHome() {
        return Commands.parallel(
            manipulator.runIntake(),
            arm.pid(ArmConstants.ballPitch, arm.getPlaceRoll()),
            elevator.pid(ElevatorConstants.ballL3),
            arm.setState(ArmState.BALL_L3_PICKUP)
        );
    }

    public Command ballRemoveL2FromHome() {
        return Commands.parallel(
            manipulator.setIntakeOnce(ManipulatorConstants.outtakePower),
            arm.pid(ArmConstants.l2BallRemovePitch, 0.0),
            elevator.pid(ElevatorConstants.ballRemoveL2),
            arm.setState(ArmState.BALL_L2_REMOVE)
        );
    }

    public Command ballRemoveL3FromHome() {
        return Commands.parallel(
            manipulator.setIntakeOnce(ManipulatorConstants.outtakePower),
            arm.pid(ArmConstants.l3BallRemovePitch, 0.0),
            elevator.pid(ElevatorConstants.ballRemoveL3),
            arm.setState(ArmState.BALL_L3_REMOVE)
        );
    }

    public Command ballPlaceFromHome() {
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

    public Command ballLaunch() {
        return Commands.sequence(
            manipulator.shoot()
        );
    }

    /**
     * back-end method
     * @param initialState inital arm state, between L1-L4
     * @return
     */
    public Command place(ArmState initialState) {
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
    public Command release(ArmState initialState) {
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
                return manipulator.outtakeUntilRelease(-0.20);
                // break;
            case L2:
                armPitch = ArmConstants.L2Pitch;

                return Commands.sequence(
                    Commands.parallel(
                        manipulator.setIntakeOnce(outtakeSpeed),
                        elevator.pid(Elevator.armStateToDownPosition(initialState)),
                        arm.pid(armReturnPitch, armRoll)
                    ),
                    Commands.parallel(
                        arm.setState(ArmState.HOME),
                        manipulator.outtakeUntilRelease(outtakeSpeed),
                        elevator.pidToBottom()
                    ),
                    Commands.waitSeconds(2.0),
                    arm.pid(ArmConstants.rotatePoint, armRoll)
                );
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
                        arm.pid(ArmConstants.downPitch, armRoll)
                    ),
                    arm.pid(ArmConstants.rotatePoint, armRoll)
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
                arm.pid(ArmConstants.downPitch, armRoll)
            ),
            arm.pid(ArmConstants.rotatePoint, armRoll)
        );
    }

    /**
     * 
     * @param targetState must be L1-L4
     * @return
     */
    public Command placeFromPlace(Arm.ArmState targetState) {
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
    public Command placeFromPlaceAuto(ArmState targetState) {
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
                elevatorPosition = ElevatorConstants.l4Auto;
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
    public Command placeFromHPAuto(Arm.ArmState targetState) {
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
                elevatorPosition = ElevatorConstants.l4Auto;
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

    
    /**
     * 
     * @param targetState must be L1-L4
     * @return
     */
    public Command placeFromHP(Arm.ArmState targetState) {
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

    public Command homeFromPlace() {
        return Commands.sequence(
                    manipulator.stopIntake(),
                    arm.pid(ArmConstants.rotatePoint, arm.getPlaceRoll()), 
                    Commands.waitSeconds(0.001),
                    arm.setState(ArmState.HOME),
                    elevator.pidToBottom()
                );
    }

    public Command homeFromBallPickup() {
        return Commands.sequence(
                    arm.pid(ArmConstants.ballStowPitch, arm.getPlaceRoll()), 
                    Commands.waitSeconds(0.001),
                    arm.setState(ArmState.HOME),
                    elevator.pidToBottom()
                );
    }

    public Command homeFromHP() {
        arm.placeLeft = manipulator.getPlaceDirection();
        return Commands.sequence(
                    manipulator.stopIntake(),
                    elevator.pidToBottom(),
                    arm.setState(ArmState.HOME),
                    arm.pid(ArmConstants.rotatePoint, arm.getPlaceRoll())
                );
    }

    public Command humanPlayerFromPlace() {
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
                    arm.pid(ArmConstants.humanPlayer, ArmConstants.humanPlayerRoll)
                ),
                manipulator.intakeUntilHolding()
            )
        );
    }

    public Command humanPlayerFromHumanPlayer() {
        return Commands.parallel(
            elevator.pid(ElevatorConstants.hp),
            arm.pid(ArmConstants.humanPlayer, ArmConstants.humanPlayerRoll),
            manipulator.intakeUntilHolding(),
            arm.setState(ArmState.HP)
        );
    }

    public Command humanPlayerFromHome() {
        return humanPlayerFromPlace();
    }

    public Command humanPlayerDown() {
        return Commands.parallel(
            elevator.pid(ElevatorConstants.hpOneCoral),
            arm.pid(ArmConstants.humanPlayerOneCoral, ArmConstants.humanPlayerRoll),
            manipulator.intakeUntilHolding(),
            arm.setState(ArmState.HP)
        );
    }

    public Command groundIntakeVerticalFromHome() {
        return Commands.parallel(
            elevator.pid(ElevatorConstants.groundIntake),
            arm.pid(ArmConstants.groundIntakeVerticalPitch, arm.getPlaceRoll()),
            manipulator.intakeUntilHolding(),
            arm.setState(ArmState.INTAKE_GROUND)
        );
    }

    public Command groundIntakeVerticalFromHumanPlayer() {
        return Commands.sequence(
            elevator.pid(ElevatorConstants.groundIntake),
            Commands.parallel(
                arm.pid(ArmConstants.groundIntakeVerticalPitch, arm.getPlaceRoll()),
                manipulator.intakeUntilHolding(),
                arm.setState(ArmState.INTAKE_GROUND)
            )
        );
    }

    public Command groundIntakeHorizontalFromHome() {
        return Commands.parallel(
            elevator.pid(ElevatorConstants.groundIntake),
            arm.pid(ArmConstants.groundIntakeHorizontalPitch, Units.degreesToRadians(45.0)),
            manipulator.intakeUntilHolding(),
            arm.setState(ArmState.INTAKE_GROUND)
        );
    }

    public Command groundIntakeHorizontalFromHumanPlayer() {
        return Commands.sequence(
            elevator.pid(ElevatorConstants.groundIntake),
            Commands.parallel(
                arm.pid(ArmConstants.groundIntakeHorizontalPitch, Units.degreesToRadians(45.0)),
                manipulator.intakeUntilHolding(),
                arm.setState(ArmState.INTAKE_GROUND)
            )
        );
    }

    public Command placingToHumanPlayer() {
        return Commands.sequence(
            Commands.parallel(
                manipulator.setIntakeOnce(-0.05),
                elevator.pid(Elevator.armStateToDownPosition(ArmState.L4)),
                Commands.sequence(
                    Commands.waitSeconds(0.200),
                    arm.pid(ArmConstants.l4ReturnPitch, arm.getPlaceRoll())
                )
            ),
            Commands.parallel(
                arm.setState(ArmState.HOME),
                manipulator.outtakeUntilRelease(-0.05),
                elevator.pidToBottom(),
                arm.pid(ArmConstants.downPitch, arm.getPlaceRoll())
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
                    arm.pid(ArmConstants.autoHumanPlayer, ArmConstants.humanPlayerRoll)
                ),
                manipulator.intakeUntilHolding()
            )
        );
    }

    public Command placingToBallPickupL2() {
        return Commands.sequence(
            Commands.parallel(
                manipulator.setIntakeOnce(-0.05),
                elevator.pid(Elevator.armStateToDownPosition(ArmState.L4)),
                Commands.sequence(
                    Commands.waitSeconds(0.200),
                    arm.pid(ArmConstants.l4ReturnPitch, arm.getPlaceRoll())
                )
            ),
            Commands.parallel(
                arm.setState(ArmState.BALL_L2_PICKUP),
                manipulator.outtakeUntilRelease(-0.05),
                elevator.pid(ElevatorConstants.ballL2),
                arm.pid(ArmConstants.ballPitch, arm.getPlaceRoll())
            ),
            manipulator.setIntakeOnce(ManipulatorConstants.intakePower)

        );
    }

}
