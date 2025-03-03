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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

public class HorizontalExtensionConstants {
    public static final int smartCurrentLimit = 40;
    public static final double ratio = 10.0;
    public static final double radius = Inches.of(1.059/2.0).in(Meters);

    public static SparkMaxConfig config = new SparkMaxConfig();
    
    static {
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(smartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0)
            .apply(new EncoderConfig().positionConversionFactor(1.0/ratio * 2.0 * Math.PI * radius)
                .velocityConversionFactor(1.0/ratio * 2.0 * Math.PI * radius * 1.0/60.0)); // to convert m/min to m/s
    }

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(1.5, 0.0, 0.01);
    public static FFCoefficients ffCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.0);

    public static Constraints constraints = new Constraints(100000.0, 100000.0);

    public static double positionTolerance = 0.02;
    public static double velocityTolerance = 1.00;

}
