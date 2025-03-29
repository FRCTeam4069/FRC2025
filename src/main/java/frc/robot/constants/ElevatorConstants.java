package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

public class ElevatorConstants {
    public static final double supplyCurrentLimit = 35.0;
    public static final double statorCurrentLimit = 40.0;
    public static final double ratio = 10.0;
    public static final double radius = Inches.of(1.855/2.0).in(Meters); // meters

    public static final TalonFXConfiguration leftConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(ArmConstants.supplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(ArmConstants.statorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ratio))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));

    public static final TalonFXConfiguration rightConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(ArmConstants.supplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(ArmConstants.statorCurrentLimit)
                .withStatorCurrentLimitEnable(true))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ratio))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
    
    // public static final Slot0Configs leftSlot0Configs = leftConfig.Slot0;
    // public static final Slot0Configs rightSlot0Configs = rightConfig.Slot0;

    // public static final MotionMagicConfigs leftMotionMagicConfig = leftConfig.MotionMagic;
    // public static final MotionMagicConfigs rightMotionMagicConfig = rightConfig.MotionMagic;

    // static {
    //     leftSlot0Configs.kP = 3.0;
    //     leftSlot0Configs.kI = 0.0;
    //     leftSlot0Configs.kD = 0.0;
    //     leftSlot0Configs.kS = 0.01;
    //     leftSlot0Configs.kV = 1.00;
    //     leftSlot0Configs.kA = 0.002;
    //     leftSlot0Configs.kG = 0.01;
    //     leftSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;

    //     leftMotionMagicConfig.MotionMagicCruiseVelocity = 9.5;
    //     leftMotionMagicConfig.MotionMagicAcceleration = 18.0;
    //     leftMotionMagicConfig.MotionMagicJerk = 1800.0;


    //     rightSlot0Configs.kP = 3.0;
    //     rightSlot0Configs.kI = 0.0;
    //     rightSlot0Configs.kD = 0.0;
    //     rightSlot0Configs.kS = 0.01;
    //     rightSlot0Configs.kV = 1.00;
    //     rightSlot0Configs.kA = 0.002;
    //     rightSlot0Configs.kG = 0.01;
    //     rightSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;

    //     rightMotionMagicConfig.MotionMagicCruiseVelocity = 9.5;
    //     rightMotionMagicConfig.MotionMagicAcceleration = 18.0;
    //     rightMotionMagicConfig.MotionMagicJerk = 1800.0;
    // }

    public volatile static PIDCoefficients pidCoefficients = new PIDCoefficients(10.0, 0.0, 0.1);
    public volatile static FFCoefficients ffCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.015);

    // public volatile static Constraints constraints = new Constraints(1.6, 4.0);
    public volatile static Constraints constraints = new Constraints(1.6, 10.0);

    public volatile static double positionTolerance = 0.02;
    public volatile static double velocityTolerance = 1.00;

    public static final double upperLimit = 1.64;
    public static final double l1 = Inches.of(0.0).in(Meters);
    public static final double l2 = 0.116;
    public static final double l3 = 0.50;
    public static final double l4 = 1.50;

    public static final double l1Down = 0.0;
    public static final double l2Down = 0.0;
    public static final double l3Down = l3 - Units.inchesToMeters(8);
    public static final double l4Down = l4 - Units.inchesToMeters(20.0);

    public static final double hp = 0.205;
    public static final double hpOneCoral = 0.402;
    public static final double ballL2 = 0.69;
    public static final double ballL3 = 1.04;
    public static final double ballPlace = upperLimit-0.02;
    public static final double groundIntake = 0.130;
    public static final double groundIntakeVertical = 0.0;

}
