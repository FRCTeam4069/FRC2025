package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

public class ElevatorConstants {
    public static final double supplyCurrentLimit = 40.0;
    public static final double statorCurrentLimit = 100.0;
    public static final double ratio = 10.0;
    public static final double radius = Inches.of(1.375/2.0).in(Meters); // meters

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

    public volatile static PIDCoefficients pidCoefficients = new PIDCoefficients(20.0, 0.0, 0.0);
    public volatile static FFCoefficients ffCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.015);

    public volatile static Constraints constraints = new Constraints(1.6, 4.0);

    public volatile static double positionTolerance = 0.02;
    public volatile static double velocityTolerance = 1.00;

    public static final double upperLimit = 1.34;
    public static final double l1 = Inches.of(1.0).in(Meters);
    public static final double l2 = Inches.of(8).in(Meters);
    public static final double l3 = Inches.of(17).in(Meters);
    public static final double l4 = 1.28;
    public static final double hp = Inches.of(20).in(Meters);

}
