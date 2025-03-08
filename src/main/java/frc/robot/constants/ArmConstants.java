package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

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
import frc.robot.constants.DrivetrainConstants.FFCoefficients;
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
                .withInverted(InvertedValue.CounterClockwise_Positive));
                

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

    public volatile static PIDCoefficients pitchPIDCoefficients = new PIDCoefficients(0.25, 0.0, 0.0);
    public volatile static PIDCoefficients rollPIDCoefficients = new PIDCoefficients(1.0, 0.0, 0.01);

    public volatile static PIDCoefficients pitchDrivePIDCoefficients = new PIDCoefficients(0.1, 0.0, 0.0);
    public volatile static PIDCoefficients rollDrivePIDCoefficients = new PIDCoefficients(0.5, 0.0, 0.0);

    public volatile static FFCoefficients pitchFFCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.038);
    public volatile static FFCoefficients rollFFCoefficients = new FFCoefficients(0.0, 0.0, 0.0, 0.7);

    public volatile static Constraints pitchConstraints = new Constraints(10.0, 50.0);
    public volatile static Constraints rollConstraints = new Constraints(6.0, 40.0);

    public volatile static double pitchPositionTolerance = Degrees.of(3.0).in(Radians);
    public volatile static double pitchVelocityTolerance = 5.0;

    public volatile static double rollPositionTolerance = Degrees.of(3.0).in(Radians);
    public volatile static double rollVelocityTolerance = 5.0;

    public static final double startingPosition = -119.3 * (Math.PI/180.0); // rads
    public static final double balancePoint = 16.0 * (Math.PI/180.0); // rads
    public static final double rotatePoint = 63.0 * (Math.PI/180.0); // rads
    public static final double humanPlayer = -90.0 * (Math.PI/180.0); // rads

    public static final double lowerLimit = -124.0 * (Math.PI/180.0); // rads
    public static final double upperLimit = 85.0 * (Math.PI/180.0); // rads

    public static final double pitchPlaceRight = 85.0 * (Math.PI/180.0); // rads

    public static final double L4Pitch = 70.0*(Math.PI/180.0); // rads

    public static boolean telemetryEnabled = false;
}