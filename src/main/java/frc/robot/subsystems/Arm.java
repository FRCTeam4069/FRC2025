// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

/**
 * Arm
 * 
 */
public class Arm extends SubsystemBase {
    private final TalonFX left;
    private final TalonFX right;
    private final CANcoder encoder;

    private DutyCycleOut leftOutput = new DutyCycleOut(0.0);
    private DutyCycleOut rightOutput = new DutyCycleOut(0.0);

    private ArmFeedforward ff = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
    private ProfiledPIDController pitchPID = new ProfiledPIDController(ArmConstants.pitchPIDCoefficients.kP(), ArmConstants.pitchPIDCoefficients.kI(), ArmConstants.pitchPIDCoefficients.kD(), ArmConstants.pitchConstraints);
    private ProfiledPIDController rollPID = new ProfiledPIDController(ArmConstants.rollPIDCoefficients.kP(), ArmConstants.rollPIDCoefficients.kI(), ArmConstants.rollPIDCoefficients.kD(), ArmConstants.rollConstraints);

    private double kGPitch = ArmConstants.pitchFFCoefficients.kG();
    private double kGRoll = ArmConstants.rollFFCoefficients.kG();

    private double lastPitch = 0.0;
    private double lastRoll = 0.0;

    public boolean placeLeft = true;

    public enum ArmState {
        HOME, // game piece loaded, arm(rotatePoint, +-90deg), elevator(0)
        L1,
        L2,
        L3,
        L4,
        HP,
        BALL_FLOOR_PICKUP,
        BALL_L2_PICKUP,
        BALL_L3_PICKUP,
        BALL_PLACE
    }

    private ArmState state = ArmState.HOME;
    public ArmState getState() {
        return state;
    }
    public Command setState(ArmState state) {
        return new InstantCommand(() -> this.state = state);
    }

    public Arm() {
        left = new TalonFX(DeviceIDs.ARM_LEFT, "rio");
        right = new TalonFX(DeviceIDs.ARM_RIGHT, "rio");
        encoder = new CANcoder(DeviceIDs.MANIPULATOR_ENCODER, "rio");

        left.getConfigurator().apply(ArmConstants.leftConfig);
        right.getConfigurator().apply(ArmConstants.rightConfig);

        MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
        encoderConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetOffset = 0.3327;
        encoder.getConfigurator().apply(encoderConfig);

        left.setPosition(Radians.of(ArmConstants.startingPosition).in(Rotations));
        right.setPosition(Radians.of(ArmConstants.startingPosition).in(Rotations));

        pitchPID.setTolerance(ArmConstants.pitchPositionTolerance, ArmConstants.pitchVelocityTolerance);
        rollPID.setTolerance(ArmConstants.rollPositionTolerance, ArmConstants.rollVelocityTolerance);

        pitchPID.setGoal(ArmConstants.startingPosition);
        rollPID.setGoal(0.0);

        SmartDashboard.putBoolean("arm telemetry", ArmConstants.telemetryEnabled);
        if (ArmConstants.telemetryEnabled) {
            SmartDashboard.putNumber("pitch kP", ArmConstants.pitchPIDCoefficients.kP());
            SmartDashboard.putNumber("pitch kI", ArmConstants.pitchPIDCoefficients.kI());
            SmartDashboard.putNumber("pitch kD", ArmConstants.pitchPIDCoefficients.kD());
            SmartDashboard.putNumber("pitch max vel", ArmConstants.pitchConstraints.maxVelocity);
            SmartDashboard.putNumber("pitch max accel", ArmConstants.pitchConstraints.maxAcceleration);

            SmartDashboard.putNumber("pitch kG", ArmConstants.pitchFFCoefficients.kG());
            SmartDashboard.putNumber("roll kG", ArmConstants.rollFFCoefficients.kG());

            SmartDashboard.putNumber("roll kP", ArmConstants.rollPIDCoefficients.kP());
            SmartDashboard.putNumber("roll kI", ArmConstants.rollPIDCoefficients.kI());
            SmartDashboard.putNumber("roll kD", ArmConstants.rollPIDCoefficients.kD());
        }
    }

    public void setLeft(double speed) {
        left.setControl(leftOutput.withOutput(speed).withLimitForwardMotion(forwardLimit()).withLimitReverseMotion(reverseLimit()));
    }

    public void setRight(double speed) {
        right.setControl(rightOutput.withOutput(speed).withLimitForwardMotion(forwardLimit()).withLimitReverseMotion(reverseLimit()));
    }

    public double getLeftRotations() {
        return left.getPosition().getValueAsDouble();
    }

    public double getRightRotations() {
        return right.getPosition().getValueAsDouble();
    }

    /**
     * @return rads/s
     */
    public double getLeftVelocity() {
        return left.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * @return rads/s
     */
    public double getRightVelocity() {
        return right.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * @return rads/s^2
     */
    public double getLeftAcceleration() {
        return left.getAcceleration().getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * @return rads/s^2
     */
    public double getRightAcceleration() {
        return right.getAcceleration().getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * @return rads, 0 is perp with ground
     */
    public double getLeftRadians() {
        return getLeftRotations() * 2 * Math.PI;
    }

    /**
     * @return rads, 0 is perp with ground
     */
    public double getRightRadians() {
        return getRightRotations() * 2 * Math.PI;
    }

    /**
     * 
     * @return rads, 0 is perp with ground
     */
    public double getPitch() {
        return (getLeftRadians() + getRightRadians()) / 2;
    }

    /**
     * 
     * @return rad/s
     */
    public double getPitchVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    /**
     * 
     * @return rad/s^2
     */
    public double getPitchAcceleration() {
        return (getLeftAcceleration() + getRightAcceleration()) / 2;
    }

    /**
     * 
     * @return rads, 0 is with manipulator down so it can pick up. CCW is positive
     */
    public double getRoll() {
        var status = encoder.getAbsolutePosition();

        if (status.getStatus().isOK()) {
            return status.getValueAsDouble() * 2.0 * Math.PI;
        } else {
            return (getLeftRadians() - getRightRadians())/2.0;
        }

    }

    /**
     * 
     * @return rad/s, CCW is positive
     */
    public double getRollVelocity() {
        return (getLeftVelocity() - getRightVelocity())/2.0;
    }

    /**
     * 
     * @return rad/s^2, CCW is positive
     */
    public double getRollAcceleration() {
        return (getLeftAcceleration() - getRightAcceleration())/2.0;
    }

    public void setLeftRight(double left, double right) {
        setLeft(left);
        setRight(right);
    }

    public void drive(double pitch, double roll) {
        var theta = getPitch() - ArmConstants.balancePoint*Math.cos(getRoll());
        var thetaRoll = getRoll();

        var ffRoll = (kGRoll * Math.sin(thetaRoll)) * (-1.0 * kGPitch * Math.sin(theta));
        var ffPitch = (-1.0 * kGPitch * Math.sin(theta)) + (ArmConstants.pitchFFCoefficients.kS() * Math.signum(pitch));

        SmartDashboard.putNumber("arm ffPitch", ffPitch);
        SmartDashboard.putNumber("arm ffRoll", ffRoll);

        var pitchOutput = pitch + ffPitch;
        var rollOutput = roll + ffRoll;

        setLeft(pitchOutput + (rollOutput/2.0));
        setRight(pitchOutput - (rollOutput/2.0));
    }

    public void setPosition(double pitch, double roll) {
        // double pitchOutput = pitchPID.calculate(getPitch(), pitch);
        // double rollOutput = rollPID.calculate(getRoll(), roll);

        pitchPID.setGoal(pitch);
        rollPID.setGoal(roll);

        //drive(pitchOutput, rollOutput);
    }

    public void goToSetpoint() {
        double pitchOutput = pitchPID.calculate(getPitch());
        double rollOutput = rollPID.calculate(getRoll());

        drive(pitchOutput, rollOutput);
    }

    public double calculatePitchPID(double pitch) {
        double pitchOutput = pitchPID.calculate(getPitch(), pitch);
        return pitchOutput;
    }

    public void resetControllers() {
        pitchPID.reset(getPitch(), getPitchVelocity());
        rollPID.reset(getRoll(), getRollVelocity());
    }

    public boolean atPosition() {
        return pitchPID.atGoal() && rollPID.atSetpoint();
    }

    public double getPlaceRoll() {
        //return placeLeft ? -ArmConstants.placeRoll : ArmConstants.placeRoll;
        return -ArmConstants.placeRoll;
    }

    public boolean canRotate() {
        return getPitch() > ArmConstants.rotatePoint;
    }

    public Command pid(double pitch, double roll) {
        var command = new Command() {
            @Override
            public void initialize() {
                resetControllers();
                setPosition(pitch, roll);
            }
            @Override
            public void execute() {
                //setPosition(pitch, roll);
            }
            @Override
            public void end(boolean interrupted) {
                stop();
            }
            @Override
            public boolean isFinished() {
                return atPosition();
            }
        };

        command.addRequirements(this);

        return command;
    }

    public Command pidRoll(double roll) {
        var command = new Command() {
            @Override
            public void initialize() {
                rollPID.reset(getRoll(), getRollVelocity());
                rollPID.setGoal(roll);
            }
            @Override
            public void execute() {

            }
            @Override
            public void end(boolean interrupted) {
                stop();
            }
            @Override
            public boolean isFinished() {
                return atPosition();
            }
        };

        command.addRequirements(this);

        return command;
    }

    public Command waitUntilHumanPlayer() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return getPitch() > -10.0;
            }
        };
    }

    // public Command keepPosition() {
    //     return run(() -> setPosition(pitchPID.getSetpoint().position, rollPID.getSetpoint().position));
    // }

    public void stop() {
        setLeft(0.0);
        setRight(0.0);
    }

    public boolean reverseLimit() {
        return getPitch() <= ArmConstants.lowerLimit;
    }

    public boolean forwardLimit() {
        return getPitch() >= ArmConstants.upperLimit;
    }

    public Command driveCommand(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> drive(left.getAsDouble(), right.getAsDouble()));
    }

    public Command rawDriveCommand(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> setLeftRight(left.getAsDouble(), right.getAsDouble()));
    }

    @Override
    public void periodic() {
        goToSetpoint();

        SmartDashboard.putString("arm state", getState().name());
        SmartDashboard.putNumber("pitch deg", getPitch()*180/Math.PI);
        SmartDashboard.putNumber("roll deg", getRoll()*180/Math.PI);
        ArmConstants.telemetryEnabled = SmartDashboard.getBoolean("arm telemetry", ArmConstants.telemetryEnabled);
        if (ArmConstants.telemetryEnabled) {
            SmartDashboard.putNumber("left", getLeftRadians());
            SmartDashboard.putNumber("right", getRightRadians());
            SmartDashboard.putNumber("pitch", getPitch());
            SmartDashboard.putNumber("roll", getRoll());
            SmartDashboard.putNumber("pitch deg", getPitch()*180/Math.PI);
            SmartDashboard.putNumber("roll deg", getRoll()*180/Math.PI);

            SmartDashboard.putNumber("pitch vel", getPitchVelocity());
            SmartDashboard.putNumber("pitch acceleration", getPitchAcceleration());
            
            SmartDashboard.putNumber("roll vel", getRollVelocity());
            SmartDashboard.putNumber("roll acceleration", getRollAcceleration());

            double pitchKP = SmartDashboard.getNumber("pitch kP", ArmConstants.pitchPIDCoefficients.kP());
            double pitchKI = SmartDashboard.getNumber("pitch kI", ArmConstants.pitchPIDCoefficients.kI());
            double pitchKD = SmartDashboard.getNumber("pitch kD", ArmConstants.pitchPIDCoefficients.kD());
            double pitchVel = SmartDashboard.getNumber("pitch max vel", ArmConstants.pitchConstraints.maxVelocity);
            double pitchAccel = SmartDashboard.getNumber("pitch max accel", ArmConstants.pitchConstraints.maxAcceleration);

            double pitchKG = SmartDashboard.getNumber("pitch kG", ArmConstants.pitchFFCoefficients.kG());

            double rollKP = SmartDashboard.getNumber("roll kP", ArmConstants.rollPIDCoefficients.kP());
            double rollKI = SmartDashboard.getNumber("roll kI", ArmConstants.rollPIDCoefficients.kI());
            double rollKD = SmartDashboard.getNumber("roll kD", ArmConstants.rollPIDCoefficients.kD());
            double rollVel = SmartDashboard.getNumber("roll max vel", ArmConstants.rollConstraints.maxVelocity);
            double rollAccel = SmartDashboard.getNumber("roll max accel", ArmConstants.rollConstraints.maxAcceleration);

            double rollKG = SmartDashboard.getNumber("roll kG", ArmConstants.rollFFCoefficients.kG());

            ArmConstants.pitchPIDCoefficients = new PIDCoefficients(pitchKP, pitchKI, pitchKD);
            pitchPID.setPID(pitchKP, pitchKI, pitchKD);

            ArmConstants.rollPIDCoefficients = new PIDCoefficients(rollKP, rollKI, rollKD);
            rollPID.setPID(rollKP, rollKI, rollKD);

            ArmConstants.pitchFFCoefficients = new FFCoefficients(0.0, pitchKG, 0.0, 0.0);
            kGPitch = pitchKG;

            ArmConstants.rollFFCoefficients = new FFCoefficients(0.0, rollKG, 0.0, 0.0);
            kGRoll = rollKG;

            ArmConstants.pitchConstraints = new Constraints(pitchVel, pitchAccel);
            ArmConstants.rollConstraints = new Constraints(rollVel, rollAccel);

        }
    }
}
