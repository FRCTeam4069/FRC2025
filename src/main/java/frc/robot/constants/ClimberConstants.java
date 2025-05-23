package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.constants.DrivetrainConstants.FFCoefficients;
import frc.robot.constants.DrivetrainConstants.PIDCoefficients;

public class ClimberConstants {
    public static final int smartCurrentLimit = 30;
    public static final int pivotCurrentLimit = 30;
    public static final double ratio = 1.6 * 7.0 * 5.0;
    public static final double pivotRatio = 25.0 * 2.333;
    public static final double radius = Inches.of(0.75/2.0).in(Meters);
    public static final double climbPos = 1.346; //rads

    public static SparkMaxConfig leftConfig = new SparkMaxConfig();
    public static SparkMaxConfig rightConfig = new SparkMaxConfig();
    public static SparkMaxConfig pivotConfig = new SparkMaxConfig();

    public static final double forwardLimit = 1.1;
    public static final double reverseLimit = 0.0;
    
    static {
        leftConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(smartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0)
            .apply(new EncoderConfig().positionConversionFactor(1.0/ratio * 2.0 * Math.PI * radius)
                .velocityConversionFactor(1.0/ratio * 2.0 * Math.PI * radius * 1.0/60.0)); // to convert m/min to m/s
        rightConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(smartCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0)
            .apply(new EncoderConfig().positionConversionFactor(1.0/ratio * 2.0 * Math.PI * radius)
                .velocityConversionFactor(1.0/ratio * 2.0 * Math.PI * radius * 1.0/60.0)); // to convert m/min to m/s
        pivotConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(pivotCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0)
            .apply(new EncoderConfig().positionConversionFactor(1.0/pivotRatio)
                .velocityConversionFactor(1.0/pivotRatio * 1.0/60.0)) // to convert m/min to m/s
            .apply(new SoftLimitConfig().forwardSoftLimit(forwardLimit).reverseSoftLimit(reverseLimit).forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true));
            
    }

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.02, 0.0, 0.0);
    public static FFCoefficients ffCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.0);

    public static Constraints constraints = new Constraints(100000.0, 100000.0);

    public static double positionTolerance = 0.02;
    public static double velocityTolerance = 1.00;

}
