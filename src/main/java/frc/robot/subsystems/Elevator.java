package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Arm.ArmState;

public class Elevator extends SubsystemBase {
    private final TalonFX left;
    private final TalonFX right;

    private DutyCycleOut leftOutput = new DutyCycleOut(0.0);
    private DutyCycleOut rightOutput = new DutyCycleOut(0.0);
    private PositionVoltage request = new PositionVoltage(0.0);

    private ProfiledPIDController pid = new ProfiledPIDController(
        ElevatorConstants.pidCoefficients.kP(), 
        ElevatorConstants.pidCoefficients.kP(), 
        ElevatorConstants.pidCoefficients.kP(), 
        ElevatorConstants.constraints);

    private ElevatorFeedforward ff = new ElevatorFeedforward(
        ElevatorConstants.ffCoefficients.kS(), 
        ElevatorConstants.ffCoefficients.kG(), 
        ElevatorConstants.ffCoefficients.kV(),
        ElevatorConstants.ffCoefficients.kA());

    private boolean lastAtBottom = false; 
    private boolean atBottom = false; 
    
    private double currentPosition = 0.0;
    private double positionOffset = 0.0;

    private final DigitalInput leftLimit;
    private final DigitalInput rightLimit;

    private double kg = ElevatorConstants.ffCoefficients.kG();
    
    public Elevator(){
        right = new TalonFX(DeviceIDs.ELEVATOR_RIGHT, "rio");
        left = new TalonFX(DeviceIDs.ELEVATOR_LEFT, "rio");

        right.getConfigurator().apply(ElevatorConstants.rightConfig);
        left.getConfigurator().apply(ElevatorConstants.leftConfig);
        // right.getConfigurator().apply(ElevatorConstants.slot0Configs);
        // left.getConfigurator().apply(ElevatorConstants.slot0Configs);
        left.setControl(new Follower(right.getDeviceID(), true));

        right.setPosition(0);
        left.setPosition(0);

        rightLimit = new DigitalInput(DeviceIDs.ELEVATOR_LIMIT_RIGHT);
        leftLimit = new DigitalInput(DeviceIDs.ELEVATOR_LIMIT_LEFT);

        pid.setTolerance(ElevatorConstants.positionTolerance, ElevatorConstants.velocityTolerance);
        
        SmartDashboard.putNumber("elevator kP", ElevatorConstants.pidCoefficients.kP());
        SmartDashboard.putNumber("elevator kI", ElevatorConstants.pidCoefficients.kI());
        SmartDashboard.putNumber("elevator kD", ElevatorConstants.pidCoefficients.kD());
        SmartDashboard.putNumber("elevator kG", ElevatorConstants.ffCoefficients.kG());
    }

    public void setPower(double power) {
        right.setControl(rightOutput.withOutput(power).withLimitReverseMotion(atBottom).withLimitForwardMotion(upperLimit()));
        // left.setControl(leftOutput.withOutput(power).withLimitReverseMotion(atBottom).withLimitForwardMotion(upperLimit()));
    }

    public void setPosition(double position) {
        // right.setControl(request.withPosition(metersToRotations(position)));
    }

    public void drive(double power) {
        var ffOutput = kg;

        setPower(power + ffOutput);
    }

    public boolean upperLimit() {
        return getPosition() > ElevatorConstants.upperLimit;
    }

    private double getRawPosition() {
        var position = rotationsToMeters((left.getPosition().getValueAsDouble() + right.getPosition().getValueAsDouble()) / 2.0);
        return position;
    }

    /**
     * @return meters
     */
    public double getPosition() {
        return currentPosition;
    }

    /**
     * @return m/s
     */
    public double getVelocity() {
        return (rotationsToMeters(left.getVelocity().getValueAsDouble()) + rotationsToMeters(right.getVelocity().getValueAsDouble())) / 2.0;
    }

    /**
     * @return m/s^2
     */
    public double getAcceleration() {
        return (rotationsToMeters(left.getAcceleration().getValueAsDouble()) + rotationsToMeters(right.getAcceleration().getValueAsDouble())) / 2.0;
    }

    /**
     * @param setpoint meters
     */
    // public void setPosition(double setpoint) {
    //     double power = pid.calculate(getPosition(), setpoint);
    //     drive(power);
    // }

    public void resetController(State state) {
        pid.reset(state);
    }

    public boolean atSetpoint() {
        if (pid.getSetpoint().position == 0.0) {
            return (atBottom || pid.atGoal());
        } else {
            return pid.atGoal();
        }
    }

    public double getSetpoint() {
        return pid.getSetpoint().position;
    }

    public static double armStateToPosition(ArmState state) {
        double pos = ElevatorConstants.l1;
        switch (state) {
            case L1:
                pos = ElevatorConstants.l1;
                break;
            case L2:
                pos = ElevatorConstants.l2;
                break;
            case L3:
                pos = ElevatorConstants.l3;
                break;
            case L4:
                pos = ElevatorConstants.l4;
                break;
            default:
                break;
        }
        return pos;
    }

    public static double armStateToDownPosition(ArmState state) {
        double pos = ElevatorConstants.l1Down;
        switch (state) {
            case L1:
                pos = ElevatorConstants.l1Down;
                break;
            case L2:
                pos = ElevatorConstants.l2Down;
                break;
            case L3:
                pos = ElevatorConstants.l3Down;
                break;
            case L4:
                pos = ElevatorConstants.l4Down;
                break;
            default:
                break;
        }
        return pos;
    }


    /**
     * @param setpoint meters
     */
    public Command pid(double setpoint) {
        return new Command() {
            @Override
            public void initialize() {
                var state = new State(getPosition(), getVelocity());
                resetController(state);
            }
            @Override
            public void execute() {
                double power = pid.calculate(getPosition(), MathUtil.clamp(setpoint, 0.0, ElevatorConstants.upperLimit));
                // if (getPosition() < 0.08) {
                //     power = MathUtil.clamp(power, -0.4, 1.0);
                // } else if (getPosition() < 0.12) {
                //     power = MathUtil.clamp(power, -0.5, 1.0);
                // }
                drive(power);
                // setPosition(setpoint);
            }
            @Override
            public boolean isFinished() {
                return atSetpoint();
            }
            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command rezero() {
        return new Command() {
            @Override
            public void execute() {
                drive(-0.3);
            }
            @Override
            public boolean isFinished() {
                return bottomPressed();
            }
            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command pidToBottom() {
        return Commands.sequence(pid(0.0), rezero());
    }

    public boolean bottomPressed() {
        return atBottom;
    }

    public double rotationsToMeters(double rotations) {
        return rotations * 2 * Math.PI * ElevatorConstants.radius;
    }

    public double metersToRotations(double meters) {
        return meters / (2 * Math.PI * ElevatorConstants.radius);
    }

    public void stop() {
        drive(0.0);
    }

    public Command drive(DoubleSupplier power) {
        return run(() -> drive(power.getAsDouble()));
    }

    @Override
    public void periodic() {
        currentPosition = getRawPosition() - positionOffset;

        atBottom = !leftLimit.get() || !rightLimit.get();

        if (atBottom && !lastAtBottom) {
            positionOffset += currentPosition;
        }

        lastAtBottom = atBottom;

        SmartDashboard.putNumber("elevator position", currentPosition);
        SmartDashboard.putNumber("left elevator position", rotationsToMeters(left.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("right elevator position", rotationsToMeters(right.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("elevator position", currentPosition);
        SmartDashboard.putBoolean("elevator limit", atBottom);
        SmartDashboard.putNumber("elevator velocity", getVelocity());
        SmartDashboard.putNumber("elevator accel", getAcceleration());
        SmartDashboard.putNumber("right elevator power", right.get());
        SmartDashboard.putNumber("left elevator power", left.get());
        SmartDashboard.putNumber("elevator setpoint", pid.getSetpoint().position);

        double kp = SmartDashboard.getNumber("elevator kP", ElevatorConstants.pidCoefficients.kP());
        double ki = SmartDashboard.getNumber("elevator kI", ElevatorConstants.pidCoefficients.kI());
        double kd = SmartDashboard.getNumber("elevator kD", ElevatorConstants.pidCoefficients.kD());
        double kg = SmartDashboard.getNumber("elevator kG", ElevatorConstants.ffCoefficients.kG());

        pid.setPID(kp, ki, kd);
        this.kg = kg;
    }
    
}