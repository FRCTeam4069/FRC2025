package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;

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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DrivetrainConstants;

public class SwerveDrivetrain extends SubsystemBase {
    private SwerveModule fl, fr, bl, br;
    private Pigeon2 gyro;
    private SwerveDriveKinematics kinematics = DrivetrainConstants.kinematics;
    private SwerveDriveOdometry poseEstimator;
    private StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveModuleStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> desiredSwervePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("DesiredSwerveModuleStates", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct).publish();
    private StructPublisher<ChassisSpeeds> speedsPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("RobotSpeeds", ChassisSpeeds.struct).publish();
    
    private SwerveModuleState[] desiredState = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    public Pose2d startingPose = new Pose2d();

    public SwerveDrivetrain() {
        fl = new SwerveModule(DrivetrainConstants.flConfig, DrivetrainConstants.flCoefficients);

        fr = new SwerveModule(DrivetrainConstants.frConfig, DrivetrainConstants.frCoefficients);

        bl = new SwerveModule(DrivetrainConstants.blConfig, DrivetrainConstants.blCoefficients);

        br = new SwerveModule(DrivetrainConstants.brConfig, DrivetrainConstants.brCoefficients);

        gyro = new Pigeon2(0, "rio");
        gyro.getConfigurator().apply(DrivetrainConstants.gyroConfig);

        poseEstimator = new SwerveDriveOdometry(kinematics, getRawRotation2d(), getModulePositions(), new Pose2d());

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
        
        SmartDashboard.putNumber("drive wheel diameter", DrivetrainConstants.wheelDiameter);
    }

    public SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volt.per(Second).of(2.0) , Volt.of(7.0), Second.of(10.0)), 
        new SysIdRoutine.Mechanism(
            this::setDriveVoltage, 
            null, 
            this));

    public SysIdRoutine steerSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volt.per(Second).of(1.0) , Volt.of(7.0), Second.of(20.0)), 
        new SysIdRoutine.Mechanism(
            this::setSteerVoltage, 
            null, 
            this));

    public Command alignForward() {
        return Commands.deadline(Commands.waitSeconds(1.0), run(() -> setDriveVoltage(Volt.of(0.0))));
    }

    public Rotation2d getRawRotation2d() {
        return gyro.getRotation2d();
    }

    public double getAngularVelocity() {
        return gyro.getAngularVelocityZWorld().getValueAsDouble() * (Math.PI/180.0);
    }

    public Rotation2d getRotation2d() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /*
    public ChassisSpeeds getBetterRobotRelativeSpeeds() {
        var realSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getRotation2d());
        var newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(realSpeeds.vxMetersPerSecond, realSpeeds.vyMetersPerSecond*DrivetrainConstants.yScalar, realSpeeds.omegaRadiansPerSecond), getRotation2d());
        
        return newSpeeds;
    }
    */

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

    public void setSteerVoltage(Voltage voltage) {
        fl.setSteerVoltage(voltage.in(Volt));
        fr.setSteerVoltage(voltage.in(Volt));
        bl.setSteerVoltage(voltage.in(Volt));
        br.setSteerVoltage(voltage.in(Volt));
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivetrainConstants.maxVelocity);
        desiredState = moduleStates;
        fl.setDesiredState(moduleStates[0]);
        fr.setDesiredState(moduleStates[1]);
        bl.setDesiredState(moduleStates[2]);
        br.setDesiredState(moduleStates[3]);
    }

    /**
     * drive with robot relatve chassis speeds. corrects for angular velocity
     * @param speeds robot relative
     */
    public void drive(ChassisSpeeds speeds) {
        speeds = angularVelocitySkewCorrection(speeds);
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void fakeDrive(ChassisSpeeds speeds) {
        speeds = angularVelocitySkewCorrection(speeds);
        var moduleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivetrainConstants.maxVelocity);
        desiredState = moduleStates;
        fl.fakeSetDesiredState(moduleStates[0]);
        fr.fakeSetDesiredState(moduleStates[1]);
        bl.fakeSetDesiredState(moduleStates[2]);
        br.fakeSetDesiredState(moduleStates[3]);
    }

    public void fieldOrientedDrive(ChassisSpeeds speeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation2d()));
    }

    public ChassisSpeeds angularVelocitySkewCorrection(ChassisSpeeds robotRelativeVelocity) {
        var rawAngularVelocity = getAngularVelocity();
        var angularVelocity = new Rotation2d(rawAngularVelocity * DrivetrainConstants.angularVelocityCoefficient);
        if (angularVelocity.getRadians() != 0.0) {
            ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, getRotation2d());
            robotRelativeVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeVelocity, getRotation2d().plus(angularVelocity));
        }

        robotRelativeVelocity = ChassisSpeeds.discretize(robotRelativeVelocity, 0.02);

        return robotRelativeVelocity;
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

    public Command stopCommand() {
        return run(() -> stop());
    }

    public void stop() {
        drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public Pose2d getPose() {
        var pose = poseEstimator.getPoseMeters();
        // return new Pose2d(pose.getX(), startingPose.getY()-DrivetrainConstants.yScalar*(startingPose.getY()-pose.getY()), pose.getRotation());
        return pose;
    }

    public void resetHeading(Rotation2d newHeading) {
        resetPose(new Pose2d(getPose().getTranslation(), newHeading));
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getRawRotation2d(), getModulePositions(), pose);
        startingPose = pose;
    }

    public Command resetHeadingCommand() {
        return runOnce(() -> resetHeading(new Rotation2d(0.0)));
    }

    @Override
    public void periodic() {
        poseEstimator.update(getRawRotation2d(), getModulePositions());

        swervePublisher.set(getModuleStates());
        desiredSwervePublisher.set(desiredState);
        posePublisher.set(getPose());
        var speeds = getRobotRelativeSpeeds();
        speedsPublisher.set(speeds);
        // SmartDashboard.putNumber("max speed", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));

        // var wheelDiameter = SmartDashboard.getNumber("drive wheel diameter", DrivetrainConstants.wheelDiameter);

        // DrivetrainConstants.wheelDiameter = wheelDiameter;

        // fl.setDriveConversionFactor(((wheelDiameter * Math.PI) * 0.0254) / 6.75);
        // fr.setDriveConversionFactor(((wheelDiameter * Math.PI) * 0.0254) / 6.75);
        // bl.setDriveConversionFactor(((wheelDiameter * Math.PI) * 0.0254) / 6.75);
        // br.setDriveConversionFactor(((wheelDiameter * Math.PI) * 0.0254) / 6.75);

    }


}
