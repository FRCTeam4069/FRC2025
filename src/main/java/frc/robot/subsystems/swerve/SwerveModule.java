package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.ModuleCoefficients;
import frc.robot.constants.DrivetrainConstants.ModuleConfig;

public class SwerveModule {
    private SparkFlex drive;
    private SparkFlex steer;
    private CANcoder encoder;
    private PIDController steerPID;
    private SimpleMotorFeedforward driveFF;

    public SwerveModule(
        ModuleConfig config,
        ModuleCoefficients coefficients
    ) {
        this(
            config.driveId(), 
            config.driveInverted(),
            config.steerId(), 
            config.steerInverted(), 
            config.encoderId(), 
            config.encoderOffset(), 
            config.encoderInverted(),
            coefficients
        );
    }

    public SwerveModule(
        int driveId, 
        boolean driveInverted,
        int steerId, 
        boolean steerInverted, 
        int encoderId, 
        double encoderOffset, 
        boolean encoderInverted,
        ModuleCoefficients moduleCoefficients
    ) {
        drive = new SparkFlex(driveId, MotorType.kBrushless);
        steer = new SparkFlex(steerId, MotorType.kBrushless);
        encoder = new CANcoder(encoderId, "rio");
        steerPID = new PIDController(moduleCoefficients.steerKP(), moduleCoefficients.steerKI(), moduleCoefficients.steerKD());
        driveFF = new SimpleMotorFeedforward(moduleCoefficients.driveKS(), moduleCoefficients.driveKV(), moduleCoefficients.driveKA());

        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig
            .inverted(driveInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DrivetrainConstants.driveCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0)
            .apply(new EncoderConfig().positionConversionFactor(DrivetrainConstants.driveConversionFactor)
                .velocityConversionFactor(DrivetrainConstants.driveConversionFactor * 1.0/60.0)); // to convert m/min to m/s
        
        driveConfig.disableVoltageCompensation();
        
        drive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig steerConfig = new SparkFlexConfig();
        steerConfig
            .inverted(steerInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DrivetrainConstants.steerCurrentLimit)
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0)
            .apply(new EncoderConfig().positionConversionFactor(DrivetrainConstants.steerConversionFactor)
                .velocityConversionFactor(DrivetrainConstants.steerConversionFactor * 1.0/60.0)); // deg/min to deg/sec

        steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
        encoderConfig.SensorDirection = encoderInverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
        encoderConfig.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetOffset = encoderOffset;
        encoder.getConfigurator().apply(encoderConfig);

        steerPID.enableContinuousInput(-Math.PI, Math.PI);

    }

    public double getEncoderRadians() {
        return getRotation2d().getRadians();
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(encoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI);
    }

    public double getDriveVelocity() {
        return drive.getEncoder().getVelocity();
    }

    public double getDrivePosition() {
        return drive.getEncoder().getPosition();
    }

    public double getSteerPosition() {
        return steer.getEncoder().getPosition();
    }

    public double getSteerVelocity() {
        return steer.getEncoder().getVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getRotation2d());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }
    
    /**
     * Main method to drive the swerve module
     * @param desiredState 
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = getRotation2d();
        
        desiredState.optimize(currentRotation);

        double driveOutput = driveFF.calculate(desiredState.speedMetersPerSecond);
        double steerOutput = steerPID.calculate(currentRotation.getRadians(), desiredState.angle.getRadians());
        drive.setVoltage(driveOutput);
        steer.set(steerOutput);

    }

    public void fakeSetDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = getRotation2d();
        desiredState.optimize(currentRotation);
        double driveOutput = driveFF.calculate(desiredState.speedMetersPerSecond);
        drive.setVoltage(driveOutput);
    }

    public void setDriveConversionFactor(double conversionFactor) {
        drive.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(conversionFactor)
                .velocityConversionFactor(conversionFactor * 1.0/60.0)), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); // to convert m/min to m/s, null, null)  
    }

    public double getSteerVoltage() {
        return steer.getAppliedOutput();
    }

    /**
     * Set the drive motor voltage and steer motor to 0rad for SysId
     * @param voltage desired voltage
     */
    public void setDriveVoltage(double voltage) {
        drive.setVoltage(voltage);
        //double steerOutput = steerPID.calculate(getRotation2d().getRadians(), 0.0);
        //steer.set(steerOutput);

    }

    public void setSteerVoltage(double voltage) {
        steer.setVoltage(voltage);
    }

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }
    
}
