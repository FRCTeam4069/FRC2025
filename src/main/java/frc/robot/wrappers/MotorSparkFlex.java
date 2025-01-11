package frc.robot.wrappers;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

public class MotorSparkFlex extends Motor {
    SparkFlex motor;
    
    // constructor
    public MotorSparkFlex(int deviceId, MotorType type) {
        motor = new SparkFlex(deviceId, type);
    }
    
    public void setPower(double power) {
        motor.set(powerLimit(power));
    }
}
