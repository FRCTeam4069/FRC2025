package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
 import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkLimitSwitch;


public class Elevator extends SubsystemBase {
    
    TalonFX leftMotor;
    TalonFX rightMotor;

    SparkLimitSwitch topLimitSwitch;
 
    private final DutyCycleOut leftOut;
    private final DutyCycleOut rightOut;

    private boolean limitState;

    public void ConfigureMotors(){
        var leftConfiguration = new TalonFXConfiguration();
        var rightConfiguration = new TalonFXConfiguration();

        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftMotor.getConfigurator().apply(leftConfiguration);
        rightMotor.getConfigurator().apply(rightConfiguration);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public Elevator() {
        leftMotor = new TalonFX(1);// TODO
        rightMotor = new TalonFX(2);

        //topLimitSwitch = TODO

        leftOut =  new DutyCycleOut(0);
        rightOut = new DutyCycleOut(0);

        limitState = topLimitSwitch.isPressed();
        
        ConfigureMotors();
     }

    void setPower(double power) {
        leftOut.Output = power;
        rightOut.Output = power;
        leftMotor.setControl(leftOut);
        rightMotor.setControl(rightOut);
    }

    double getPosition() {
        return 0;
    }

    void stop(){
        setPower(0);
    }

    @Override
    public void periodic(){

        if (topLimitSwitch.isPressed() && !limitState) {
            stop();
        }

        limitState = topLimitSwitch.isPressed();
    }
}


