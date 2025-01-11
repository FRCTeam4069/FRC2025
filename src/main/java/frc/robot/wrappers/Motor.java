package frc.robot.wrappers;

import com.revrobotics.spark.SparkMax;      // can't find cansparkmax
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase;


abstract class Motor {
    public enum Controller {MAX, FLEX};      // anymore, should it be in constants?

    private SparkBase motor;

    double maxPower = 0.5;               // ?

    // set power
    void set (double power) {
        motor.set(Math.max(-maxPower, power));
    }
}
