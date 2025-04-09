// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
    private final TalonFX motor;
    private final DutyCycleOut output = new DutyCycleOut(0.0);
    private LaserCan leftSensor;
    private LaserCan rightSensor;
    private double left = 10000;
    private double right = 10000;

    public Manipulator() {
        motor = new TalonFX(DeviceIDs.MANIPULATOR, "rio");

        motor.getConfigurator().apply(ManipulatorConstants.talonConfig);

        motor.setPosition(0.0);

        leftSensor = new LaserCan(DeviceIDs.MANIPULATOR_LEFT_LASER_CAN);
        rightSensor = new LaserCan(DeviceIDs.MANIPULATOR_RIGHT_LASER_CAN);

        Alert alert = new Alert("LaserCAN", AlertType.kWarning);

        try {
            leftSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            leftSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            leftSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);

            rightSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            rightSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            rightSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
            alert.set(true);
            alert.setText("LaserCAN bad" + e.getMessage());
        }

        alert.close();
    }

    private double getLeftSensor() {
        // return 10000.0;
        Measurement measurement = leftSensor.getMeasurement();
        if (measurement == null) {
            return left;
        }

        if (measurement.status == LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS || measurement.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL || measurement.status == LaserCan.LASERCAN_STATUS_NOISE_ISSUE) {
            return 10000;
        }

        if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        } 

        return left;
    }

    private double getRightSensor() {
        Measurement measurement = rightSensor.getMeasurement();
        if (measurement == null) {
            return right;
        }

        if (measurement.status == LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS || measurement.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL || measurement.status == LaserCan.LASERCAN_STATUS_NOISE_ISSUE) {
            return 10000;
        }

        if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        } 

        return right;
    }

    public boolean isLeft() {
        return left < 8.0;
    }

    public boolean isRight() {
        return right < 8.0;
    }

    public double getLeftDistance() {
        return left;
    }

    public double getRightDistance() {
        return right;
    }

    public boolean isEmpty() {
        return (getLeftDistance() > ManipulatorConstants.emptyDistance) && (getRightDistance() > ManipulatorConstants.emptyDistance);
    }

    public boolean isFull() {
        return isLeft() || isRight();
    }

    public boolean isCentered() {
        return isLeft() && isRight();
    }

    /**
     * 
     * @return true if left
     */
    public boolean getPlaceDirection() {
        if (isLeft() && !isRight()) {
            return true;
        }
        return false;
    }

    public void setIntake(double power) {
        motor.setControl(output.withOutput(power));
    }

    public void stop() {
        motor.stopMotor();
    }

    public Command intakeUntilHolding() {
        return Commands.sequence(
            new InstantCommand(() -> setIntake(ManipulatorConstants.intakePower)),
            Commands.waitUntil(() -> isFull()),
            Commands.waitSeconds(0.1),
            new InstantCommand(() -> stop())
        );
    }

    public Command intakeUntilDetect() {
        return new Command() {
            @Override
            public void execute() {
                setIntake(ManipulatorConstants.intakePower);
            }

            @Override
            public boolean isFinished() {
                return isLeft() || isRight();
            }

            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }

    public Command outtakeUntilRelease() {
        return new Command() {
            @Override
            public void execute() {
                setIntake(ManipulatorConstants.outtakePower);
            }

            @Override
            public boolean isFinished() {
                return isEmpty();
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command outtakeUntilRelease(double speed) {
        return new Command() {
            @Override
            public void execute() {
                setIntake(speed);
            }

            @Override
            public boolean isFinished() {
                return isEmpty();
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    /**
     * set intake to intakePower once
     */ 
    public Command runIntake() {
        return runOnce(() -> setIntake(ManipulatorConstants.intakePower));
    }

    public Command runIntakeMax() {
        return runOnce(() -> setIntake(0.95));
    }

    public Command stopIntake() {
        return runOnce(() -> setIntake(0.0));
    }

    public Command setIntakeOnce(double speed) {
        return runOnce(() -> setIntake(speed));
    }

    public Command defaultCommand(DoubleSupplier power) {
        return new Command() {
            @Override
            public void execute() {
                
            }
        };
    }

    public Command shoot() {
        return new Command() {
            @Override
            public void execute() {
                setIntake(-1.0);
            }

            @Override
            public boolean isFinished() {
                return isEmpty();
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    @Override
    public void periodic() {
        left = getLeftSensor();
        right = getRightSensor();

        SmartDashboard.putNumber("left sensor", getLeftDistance());
        SmartDashboard.putNumber("right sensor", getRightDistance());
    }
}
