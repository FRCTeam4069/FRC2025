package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DrivetrainConstants;

public class SwerveDrivetrain extends SubsystemBase {
    private SwerveModule fl, fr, bl, br;
    private Pigeon2 gyro;
    private SwerveDriveKinematics kinematics = DrivetrainConstants.kinematics;
    private SwerveDrivePoseEstimator poseEstimator;
    private StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveModuleStates", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct).publish();
    private StructPublisher<ChassisSpeeds> speedsPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("RobotSpeeds", ChassisSpeeds.struct).publish();

    public SwerveDrivetrain() {
        fl = new SwerveModule(DrivetrainConstants.flConfig, DrivetrainConstants.flCoefficients);

        fr = new SwerveModule(DrivetrainConstants.frConfig, DrivetrainConstants.frCoefficients);

        bl = new SwerveModule(DrivetrainConstants.blConfig, DrivetrainConstants.blCoefficients);

        br = new SwerveModule(DrivetrainConstants.brConfig, DrivetrainConstants.brCoefficients);

        gyro = new Pigeon2(0, "rio");
        gyro.getConfigurator().apply(DrivetrainConstants.gyroConfig);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRawRotation2d(), getModulePositions(), new Pose2d());

        try{
            DrivetrainConstants.config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose, 
            this::resetPose, 
            this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> drive(speeds), 
            new PPHolonomicDriveController(DrivetrainConstants.translationPIDConstants, DrivetrainConstants.rotationPIDConstants), 
            DrivetrainConstants.config, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);
        
    }

    public SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
            this::setDriveVoltage, 
            null, 
            this));

    public Rotation2d getRawRotation2d() {
        return gyro.getRotation2d();
    }

    public Rotation2d getRotation2d() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            fl.getModulePosition(),
            fr.getModulePosition(),
            bl.getModulePosition(),
            br.getModulePosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            fl.getState(),
            fr.getState(),
            bl.getState(),
            br.getState()
        };
    }

    public void setDriveVoltage(Voltage voltage) {
        fl.setDriveVoltage(voltage.magnitude());
        fr.setDriveVoltage(voltage.magnitude());
        bl.setDriveVoltage(voltage.magnitude());
        br.setDriveVoltage(voltage.magnitude());
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivetrainConstants.maxVelocity);
        fl.setDesiredState(moduleStates[0]);
        fr.setDesiredState(moduleStates[1]);
        bl.setDesiredState(moduleStates[2]);
        br.setDesiredState(moduleStates[3]);
    }

    public void drive(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void fieldOrientedDrive(ChassisSpeeds speeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation()));
    }

    /**
     * Drive the robot field centric. Because joysticks have a different axis 
     * convention, all inputs need to be negative and the x and y axis need 
     * to be swapped. (this is intentional)
     * @param xVelocity velocity in the x axis
     * @param yVelocity velocity in the y axis
     * @param angleVelocity angular velocity around z axis (ccw positive)
     * @return Command
     */
    public Command driveCommand(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier angleVelocity) {
        return run(() -> {
            fieldOrientedDrive(
                new ChassisSpeeds(
                    Math.pow(xVelocity.getAsDouble(), 3) * DrivetrainConstants.maxVelocity,
                    Math.pow(yVelocity.getAsDouble(), 3) * DrivetrainConstants.maxVelocity,
                    Math.pow(angleVelocity.getAsDouble(), 3) * DrivetrainConstants.maxAngularVelocity));
        });
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getRawRotation2d(), getModulePositions(), pose);
    }

    public void resetHeading() {
        resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Command resetHeadingCommand() {
        return runOnce(() -> resetHeading());
    }

    @Override
    public void periodic() {
        poseEstimator.update(getRawRotation2d(), getModulePositions());

        swervePublisher.set(getModuleStates());
        posePublisher.set(getPose());
        var speeds = getRobotRelativeSpeeds();
        speedsPublisher.set(speeds);
        SmartDashboard.putNumber("max speed", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));

    }


}
