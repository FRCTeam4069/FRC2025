package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

public class ManipulatorConstants {
    public static final int smartCurrentLimit = 40;
    public static final double intakeRatio = 5.0;
    public static final double kickerRatio = 25.0;
    public static final double kickerRadius = Inches.of(1.059/2.0).in(Meters);
    public static final double intakeRadius = Inches.of(1.059/2.0).in(Meters);
    public static final double intakePower = 0.80;
    public static final double outtakePower = -0.50;
    public static final double intakeHoldPower = 0.5;
    public static final double kickerPlacePower = 1.0;
    public static final double kickerPlaceTime = 1.3;
    public static final double emptyDistance = 22.0; // mm
    // 1.875in edge to edge white wheel minimum distance
    public static final double supplyCurrentLimit = 40.0;
    public static final double statorCurrentLimit = 40.0;
    
    public static final TalonFXConfiguration talonConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(supplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(statorCurrentLimit)
            .withStatorCurrentLimitEnable(true))
    .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(10.0 * 1.22))
    .withMotorOutput(
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive));

    // public static SparkMaxConfig intakeConfig = new SparkMaxConfig();
    // public static SparkMaxConfig kickerConfig = new SparkMaxConfig();
    
    // static {
    //     intakeConfig
    //         .inverted(true)
    //         .idleMode(IdleMode.kBrake)
    //         .smartCurrentLimit(smartCurrentLimit)
    //         .openLoopRampRate(0.0)
    //         .closedLoopRampRate(0.0)
    //         .apply(new EncoderConfig().positionConversionFactor(1.0/intakeRatio * 2.0 * Math.PI)
    //             .velocityConversionFactor(1.0/intakeRatio * 2.0 * Math.PI * 1.0/60.0)); // to convert rad/min to rad/s

    //     kickerConfig
    //         .inverted(true)
    //         .idleMode(IdleMode.kBrake)
    //         .smartCurrentLimit(smartCurrentLimit)
    //         .openLoopRampRate(0.0)
    //         .closedLoopRampRate(0.0)
    //         .apply(new EncoderConfig().positionConversionFactor(1.0/kickerRatio * 2.0 * Math.PI)
    //             .velocityConversionFactor(1.0/kickerRatio * 2.0 * Math.PI * 1.0/60.0)); // to convert rad/min to rad/s
    // }

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.0, 0.0, 0.0);
    public static FFCoefficients ffCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.0);

    public static double positionTolerance = 0.02;
    public static double velocityTolerance = 1.00;

}
