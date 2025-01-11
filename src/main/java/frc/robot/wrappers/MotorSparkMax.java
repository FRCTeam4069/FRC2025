package frc.robot.wrappers;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class MotorSparkMax extends Motor {
    SparkMax motor;
    
    // constructor
    public MotorSparkMax(int deviceId, MotorType type) {
        motor = new SparkMax(deviceId, type);
    }
    
    public void setPower(double power) {
        motor.set(powerLimit(power));
    }
}
