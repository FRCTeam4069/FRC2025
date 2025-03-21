package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {

    public static final int driveCurrentLimit = 50;
    public static final int steerCurrentLimit = 30;
    public volatile static double wheelDiameter = 3.99; // 3.94in
    //bl 3.955 -> 3.912
    //fl 3.9545
    //fr 3.950
    //br 3.945
    public static final double driveConversionFactor = ((wheelDiameter * Math.PI) * 0.0254) / 6.75;
    public static final double steerConversionFactor = 16.8;

    public static final boolean driveInverted = true;
    public static final boolean steerInverted = true;
    public static final boolean encoderInverted = true;

    public static final double flEncoderOffset = -0.179932;
    public static final double frEncoderOffset = 0.48364295;
    public static final double blEncoderOffset = 0.425049;
    public static final double brEncoderOffset = -0.4025;

    public static final Pigeon2Configuration gyroConfig = new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(0.0));

    public static final double moduleOffset = Units.inchesToMeters(10.375);
    
    public static final double maxVelocity = Units.feetToMeters(17.6);
    public static final double maxAngularVelocity = maxVelocity / new Rotation2d(moduleOffset, moduleOffset).getRadians();

    public static final double angularVelocityCoefficient = 0.04;
    public static final double angularVelocityDeadband = 0.01;
    public static final double headingCorrectionDeadband = 0.05;

    public static final double humanPlayerLeft = Degrees.of(-52.5).in(Radians);
    public static final double humanPlayerRight = Degrees.of(52.5).in(Radians);

    public static final double mass = Pounds.of(135.0).in(Kilograms);

    public static final Rotation2d[] snapAngles = new Rotation2d[]{
        Rotation2d.fromDegrees(0.0),
        Rotation2d.fromDegrees(60.0),
        Rotation2d.fromDegrees(120.0),
        Rotation2d.fromDegrees(180.0),
        Rotation2d.fromDegrees(-120.0),
        Rotation2d.fromDegrees(-60.0)
    };

    public static final Pose2d[] blueLeftReefPoses = new Pose2d[]{
        new Pose2d(3.68, 3.03, Rotation2d.fromDegrees(60.0)),
        new Pose2d(3.22, 4.21, Rotation2d.fromDegrees(0.0)),
        new Pose2d(4.02, 5.205, Rotation2d.fromDegrees(-60.0)),
        new Pose2d(5.02, 5.17, Rotation2d.fromDegrees(-120.0)),
        new Pose2d(5.75, 4.16, Rotation2d.fromDegrees(180.0)),
        new Pose2d(5.25, 3.03, Rotation2d.fromDegrees(120.0))
    };

    public static final Pose2d[] blueRightReefPoses = new Pose2d[]{
        new Pose2d(4.00, 2.87, Rotation2d.fromDegrees(60.0)),
        new Pose2d(3.24, 3.88, Rotation2d.fromDegrees(0.0)),
        new Pose2d(3.72, 5.02, Rotation2d.fromDegrees(-60.0)),
        new Pose2d(5.30, 5.03, Rotation2d.fromDegrees(-120.0)),
        new Pose2d(5.76, 3.85, Rotation2d.fromDegrees(180.0)),
        new Pose2d(4.97, 2.83, Rotation2d.fromDegrees(120.0))
    };

    public static final Pose2d[] redLeftReefPoses = new Pose2d[]{
        new Pose2d(3.68, 3.03, Rotation2d.fromDegrees(60.0)),
        new Pose2d(3.22, 4.21, Rotation2d.fromDegrees(0.0)),
        new Pose2d(4.02, 5.205, Rotation2d.fromDegrees(-60.0)),
        new Pose2d(5.30, 5.03, Rotation2d.fromDegrees(-120.0)),
        new Pose2d(5.76, 3.85, Rotation2d.fromDegrees(180.0)),
        new Pose2d(4.97, 2.83, Rotation2d.fromDegrees(120.0))
    };

    public static final Pose2d[] redRightReefPoses = new Pose2d[]{
        new Pose2d(4.00, 2.87, Rotation2d.fromDegrees(60.0)),
        new Pose2d(3.24, 3.88, Rotation2d.fromDegrees(0.0)),
        new Pose2d(3.72, 5.02, Rotation2d.fromDegrees(-60.0)),
        new Pose2d(5.02, 5.17, Rotation2d.fromDegrees(-120.0)),
        new Pose2d(5.75, 4.16, Rotation2d.fromDegrees(180.0)),
        new Pose2d(5.25, 3.03, Rotation2d.fromDegrees(120.0))
    };

    // offset = 0.164m

    public static RobotConfig config;

    public static PIDConstants translationPIDConstants = new PIDConstants(7.0, 0.0, 0.0);
    public static PIDConstants rotationPIDConstants = new PIDConstants(4.0, 0.0, 0.0);

    public static double yScalar = 0.93;

    /*
     * 26ft 5in x 57ft 6(7/8)in
     * 8.052m x 17.548m
     */

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(moduleOffset, moduleOffset),
        new Translation2d(moduleOffset, -moduleOffset),
        new Translation2d(-moduleOffset, moduleOffset),
        new Translation2d(-moduleOffset, -moduleOffset)
    );

    public record PIDCoefficients (
        double kP,
        double kI,
        double kD
    ) {}

    public record FFCoefficients (
        double kS,
        double kV,
        double kA,
        double kG
    ) {}

    public record Tolerances (
        double position,
        double velocity
    ) {}

    public record DrivetrainPIDConstants (
        PIDCoefficients translationCoefficients,
        PIDCoefficients rotationCoefficients,
        Constraints translationConstraints,
        Constraints rotationConstraints,
        Tolerances translationTolerances,
        Tolerances rotationTolerances
    ) {}

    public static volatile DrivetrainPIDConstants pidToPositionConstants = new DrivetrainPIDConstants(
        new PIDCoefficients(8.0, 0.0, 0.0), 
        new PIDCoefficients(10.0, 0.0, 0.4), 
        new Constraints(5.0, 3.0), 
        new Constraints(10.0, 10.0), 
        new Tolerances(0.01, 0.20), 
        new Tolerances(0.02, 0.20));

    public static volatile DrivetrainPIDConstants autoPidToPositionConstants = new DrivetrainPIDConstants(
        new PIDCoefficients(6.0, 0.0, 0.0), 
        new PIDCoefficients(8.0, 0.0, 0.6), 
        new Constraints(5.0, 3.0), 
        new Constraints(10.0, 10.0), 
        new Tolerances(0.01, 0.20), 
        new Tolerances(0.02, 0.20));

    public static final PIDCoefficients teleOpHeadingCoefficients = new PIDCoefficients(13.0, 0.0, 0.0);

    public record ModuleCoefficients(
        double steerKS,
        double steerKV,
        double steerKA,

        double steerKP,
        double steerKI,
        double steerKD,

        double driveKS,
        double driveKV,
        double driveKA,

        double driveKP,
        double driveKI,
        double driveKD
    ) {}

    public static ModuleCoefficients flCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.55,
        0.0,
        0.0,

        // 0.061811,
        // 2.311,
        // 0.23706,

        // 0.07424,
        // 2.2829,
        // 0.39404,

        0.47,
        2.20,
        0.20,
        
        0.0,
        0.0,
        0.0
    );

    public static ModuleCoefficients frCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.55,
        0.0,
        0.0,

        // 0.061811,
        // 2.3088,
        // 0.26326,

        // 0.25117,
        // 2.2867,
        // 0.14551,

        0.01,
        2.14,
        0.20,
        
        0.0,
        0.0,
        0.0
    );

    public static ModuleCoefficients blCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.55,
        0.0,
        0.0,

        // 0.066838,
        // 2.3095,
        // 0.25683,

        // 0.011768,
        // 2.3304,
        // 0.35736,

        0.17,
        2.18,
        0.20,
        
        0.0,
        0.0,
        0.0
    );

    public static ModuleCoefficients brCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.55,
        0.0,
        0.0,

        // 0.081577,
        // 2.3094,
        // 0.23706,

        // 0.12165,
        // 2.2853,
        // 0.30262,

        0.00,
        2.14,
        0.20,
        
        0.0,
        0.0,
        0.0
    );

    public record ModuleConfig(
        int driveId, 
        boolean driveInverted,
        int steerId, 
        boolean steerInverted, 
        int encoderId, 
        double encoderOffset, 
        boolean encoderInverted
    ) {};

    public static final ModuleConfig flConfig = new ModuleConfig(
        DeviceIDs.DRIVE_FL, DrivetrainConstants.driveInverted, 
        DeviceIDs.STEER_FL, DrivetrainConstants.steerInverted, 
        DeviceIDs.ENCODER_FL, DrivetrainConstants.flEncoderOffset, DrivetrainConstants.encoderInverted
    );

    public static final ModuleConfig frConfig = new ModuleConfig(
        DeviceIDs.DRIVE_FR, DrivetrainConstants.driveInverted, 
        DeviceIDs.STEER_FR, DrivetrainConstants.steerInverted, 
        DeviceIDs.ENCODER_FR, DrivetrainConstants.frEncoderOffset, DrivetrainConstants.encoderInverted
    );

    public static final ModuleConfig blConfig = new ModuleConfig(
        DeviceIDs.DRIVE_BL, DrivetrainConstants.driveInverted, 
        DeviceIDs.STEER_BL, DrivetrainConstants.steerInverted, 
        DeviceIDs.ENCODER_BL, DrivetrainConstants.blEncoderOffset, DrivetrainConstants.encoderInverted
    );

    public static final ModuleConfig brConfig = new ModuleConfig(
        DeviceIDs.DRIVE_BR, DrivetrainConstants.driveInverted, 
        DeviceIDs.STEER_BR, DrivetrainConstants.steerInverted, 
        DeviceIDs.ENCODER_BR, DrivetrainConstants.brEncoderOffset, DrivetrainConstants.encoderInverted
    );
}
