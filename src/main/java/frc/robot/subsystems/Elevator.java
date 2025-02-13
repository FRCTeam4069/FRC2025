package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.Reset;

import com.ctre.phoenix6.hardware.TalonFX;

import java.beans.Encoder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkLimitSwitch;

import frc.robot.constants.DeviceIDs;
 

public class Elevator extends SubsystemBase {
    
    private TalonFX leftMotor;
    private TalonFX rightMotor;


    private final DutyCycleOut leftOut;
    private final DutyCycleOut rightOut;

    private boolean limitState; 

    final DigitalInput reverseSoftLimit = new DigitalInput(1);//FIXME once wwe know the actual port

    
    public void ConfigureMotors(){

        var leftConfiguration = new TalonFXConfiguration();
        var rightConfiguration = new TalonFXConfiguration();

        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rightConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        leftMotor.getConfigurator().apply(leftConfiguration);
        rightMotor.getConfigurator().apply(rightConfiguration);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
        
    }

    public Elevator() {
        leftMotor = new TalonFX(1);// FIXME
        rightMotor = new TalonFX(2);//FIXME

        leftOut =  new DutyCycleOut(0);
        rightOut = new DutyCycleOut(0);

        // TODO put a variable in the device ID's thing for all of this

        limitState = reverseLimitSwitch.isPressed();
        
        ConfigureMotors();
    }

    void setPower(double power) {
        leftOut.Output = power;
        rightOut.Output = power;
        leftMotor.setControl(leftOut);
        rightMotor.setControl(rightOut);
    }

    double getPosition() {
        return leftMotor.getPosition().getValueAsDouble(); // TODO math to inches or metres
    }

    void stop(){
        setPower(0);
    }

    @Override
    public void periodic(){

        if (limitState=true) {
            stop();

           // leftMotor.setPosition(1); //reset encoders
           // rightMotor.setPosition(1);
           // TODO rising state detector
           //TODO reset encoders like peters code
           //TODO trapezoidal motion thing implementation
        }

        
        leftMotor.setControl(leftOut.withOutput(1.0).withLimitReverseMotion(reverseSoftLimit.get()));
        rightMotor.setControl(rightOut.withOutput(1.0).withLimitReverseMotion(reverseSoftLimit.get()));
  
        
    }
    
//TODO power limits for motors
//TODO clamp power spike for when it hits in case of failure

//TODO trapezoidal motion 
//TODO

}


//TODO pid loop integration for gravity feedforward