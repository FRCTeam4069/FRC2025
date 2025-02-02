package frc.robot.constants;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {

    public static final int driveCurrentLimit = 60;
    public static final int steerCurrentLimit = 30;
    public static final double driveConversionFactor = ((3.979591837 * Math.PI) * 0.0254) / 6.12;
    public static final double steerConversionFactor = 16.8;

    public static final boolean driveInverted = true;
    public static final boolean steerInverted = true;
    public static final boolean encoderInverted = true;

    public static final double flEncoderOffset = 118.379/360;
    public static final double frEncoderOffset = 154.512/360;
    public static final double blEncoderOffset = -149.854/360;
    public static final double brEncoderOffset = 169.453/360;

    public static final Pigeon2Configuration gyroConfig = new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(0.0));

    public static final double moduleOffset = Units.inchesToMeters(10.375);
    
    public static final double maxVelocity = Units.feetToMeters(19.3);
    public static final double maxAngularVelocity = maxVelocity / new Rotation2d(moduleOffset, moduleOffset).getRadians();

    public static RobotConfig config;

    public static PIDConstants translationPIDConstants = new PIDConstants(0.5, 0.0, 0.0);
    public static PIDConstants rotationPIDConstants = new PIDConstants(0.1, 0.0, 0.0);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(moduleOffset, moduleOffset),
        new Translation2d(moduleOffset, -moduleOffset),
        new Translation2d(-moduleOffset, moduleOffset),
        new Translation2d(-moduleOffset, -moduleOffset)
    );

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

        0.6,
        0.0,
        0.0,

        0.2,
        2.03,
        0.43,
        
        0.0,
        0.0,
        0.0
    );

    public static ModuleCoefficients frCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.6,
        0.0,
        0.0,

        0.2,
        2.03,
        0.43,
        
        0.0,
        0.0,
        0.0
    );

    public static ModuleCoefficients blCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.6,
        0.0,
        0.0,

        0.2,
        2.03,
        0.43,
        
        0.0,
        0.0,
        0.0
    );

    public static ModuleCoefficients brCoefficients = new ModuleCoefficients(
        0.0,
        0.0,
        0.0,

        0.6,
        0.0,
        0.0,

        0.2,
        2.03,
        0.43,
        
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
