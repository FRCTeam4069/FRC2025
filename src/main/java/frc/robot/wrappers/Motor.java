package frc.robot.wrappers;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.revrobotics.spark.SparkBase;


abstract class Motor {
    double maxPower = 0.5;

    public double powerLimit(double p) {
        return Math.max(Math.min(p, maxPower), -maxPower);
    }
}
