// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {

    SparkFlex motor;

    /** Creates a new TestSubsystem. */
    public TestSubsystem() {

        motor = new SparkFlex(0, SparkFlex.MotorType.kBrushless);

    }

    public void setPower(double power) {
        motor.set(power);
    }

    public double getPosition() {
        // read encoder
        return motor.getEncoder().getPosition();
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
