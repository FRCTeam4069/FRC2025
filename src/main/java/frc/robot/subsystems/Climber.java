package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.constants.ClimberConstants;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.HorizontalExtensionConstants;

public class Climber extends SubsystemBase {
    private final SparkMax left;
    private final SparkMax right;
    private final SparkMax pivot;

    private double currentPosition = 0.0;
    private double positionOffset = 0.0;

    private PIDController pid = new PIDController(
        ClimberConstants.pidCoefficients.kP(), 
        ClimberConstants.pidCoefficients.kI(), 
        ClimberConstants.pidCoefficients.kD());
    
    public Climber() {
        left = new SparkMax(DeviceIDs.CLIMBER_LEFT, MotorType.kBrushless);
        right = new SparkMax(DeviceIDs.CLIMBER_RIGHT, MotorType.kBrushless);
        pivot = new SparkMax(DeviceIDs.CLIMBER_PIVOT, MotorType.kBrushless);

        left.configure(ClimberConstants.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        right.configure(ClimberConstants.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivot.configure(ClimberConstants.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        left.getEncoder().setPosition(0.0);
        right.getEncoder().setPosition(0.0);
        pivot.getEncoder().setPosition(0.0);

        pid.setTolerance(ClimberConstants.positionTolerance, ClimberConstants.velocityTolerance);
    }

    public void setPower(double power) {
        if (getPosition() < ClimberConstants.climbPos) {
            power = MathUtil.clamp(power, -1.0, 0.0);
        }
        left.set(power);
        right.set(power);
    }

    public void setPivot(double power) {
        pivot.set(power);
    }

    /**
     * 
     * @return rads
     */
    private double getRawPosition() {
        var position = pivot.getEncoder().getPosition() * Math.PI * 2.0;
        return position;
    }

    /**
     * 
     * @return rads
     */
    public double getPosition() {
        return getRawPosition();
    }

    public double getVelocity() {
        return left.getEncoder().getVelocity();
    }

    public void stop() {
        setPower(0);
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    public void stopPivot() {
        setPivot(0);
    }

    public Command stopPivotCommand() {
        return runOnce(() -> stopPivot());
    }

    public Command drive(DoubleSupplier power) {
        return run(() -> setPower(power.getAsDouble()));
    }

    public Command drivePivot(DoubleSupplier power) {
        return run(() -> setPivot(power.getAsDouble()));
    }

    /**
     * 
     * @param setpoint rads
     * @return
     */
    public Command pid(double setpoint) {
        return new Command() {
            @Override
            public void initialize() {
                pid.calculate(getPosition(), setpoint);
            }

            @Override
            public void execute() {
                setPivot(pid.calculate(getPosition(), setpoint));
            }

            @Override
            public boolean isFinished() {
                return pid.atSetpoint();
            }

            @Override
            public void end(boolean interrupted) {
                stopPivot();
            }
        };
    }

    public Command filp() {
        return new Command() {
            @Override
            public void initialize() {
                
            }

            @Override
            public void execute() {
                if (getPosition() > 2.0) {
                    setPivot(0.25);
                } else {
                    setPivot(0.75);
                }
            }

            @Override
            public boolean isFinished() {
                return getPosition() >= 3.30;
            }

            @Override
            public void end(boolean interrupted) {
                stopPivot();
            }
        };
    }

    public Command winch() {
        return new Command() {
            @Override
            public void initialize() {
                
            }

            @Override
            public void execute() {
                setPower(1.0);
            }

            @Override
            public boolean isFinished() {
                return getPosition() <= ClimberConstants.climbPos;
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command defaultCommand(DoubleSupplier power, BooleanSupplier enable) {
        return Commands.either(drive(power), stopCommand(), enable);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("climber left pos", left.getEncoder().getPosition());
        SmartDashboard.putNumber("climber right pos", right.getEncoder().getPosition());
        SmartDashboard.putNumber("climber pivot pos", pivot.getEncoder().getPosition());
        SmartDashboard.putNumber("climber pos", getPosition());
        SmartDashboard.putNumber("climber pivot pos deg", Units.radiansToDegrees(getPosition()));
        SmartDashboard.putNumber("climber pivot power", pivot.get());
        SmartDashboard.putNumber("climber power", left.get());
        SmartDashboard.putNumber("climber velocity", getVelocity());

    }
    
}