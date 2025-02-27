package frc.robot.constants;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;
public class ArmConstants {

    public static final double supplyCurrentLimit = 40.0;
    public static final double statorCurrentLimit = 100.0;
    public static final double ratio = 5.0 * 5.0;

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
                .withInverted(InvertedValue.Clockwise_Positive));
                

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
                .withInverted(InvertedValue.CounterClockwise_Positive));

    public static PIDCoefficients pitchPIDCoefficients = new PIDCoefficients(0.5, 0.0, 0.001);
    public static PIDCoefficients rollPIDCoefficients = new PIDCoefficients(1.5, 0.0, 0.01);

    public static Constraints pitchConstraints = new Constraints(100000.0, 100000.0);
    public static Constraints rollConstraints = new Constraints(100000.0, 100000.0);

    public static double pitchPositionTolerance = 0.02;
    public static double pitchVelocityTolerance = 1.00;

    public static double rollPositionTolerance = 0.02;
    public static double rollVelocityTolerance = 1.00;

    public static final double startingPosition = 122.0 * (Math.PI/180.0); // rads
    public static final double balancePoint = -25.0 * (Math.PI/180.0); // rads
}