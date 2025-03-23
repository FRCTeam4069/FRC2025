package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class FieldCentricDrive extends Command {
    private final SwerveDrivetrain drive;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier turnSpeed;
    private final BooleanSupplier halfSpeed;
    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(100.0);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(100.0);
    private PIDController headingController = new PIDController(
        DrivetrainConstants.teleOpHeadingCoefficients.kP(), 
        DrivetrainConstants.teleOpHeadingCoefficients.kI(), 
        DrivetrainConstants.teleOpHeadingCoefficients.kD());
    private BooleanSupplier headingCorrection;
    private BooleanSupplier humanPlayerLeft;
    private BooleanSupplier humanPlayerRight;
    private BooleanSupplier snapHeading;
    private double lastHeading = 0.0;
    private Alliance alliance = Alliance.Blue;

    private static double controllerDeadband = 0.05;
    /**
     * Teleop drive command
     * @param drive swerve drivetrain
     * @param forwardSpeed -1.0 to 1.0
     * @param strafeSpeed -1.0 to 1.0
     * @param turnSpeed -1.0 to 1.0
     * @param halfSpeed 
     * @param headingCorrection whether to run pid controller on the heading
     */
    public FieldCentricDrive(
        SwerveDrivetrain drive, 
        DoubleSupplier forwardSpeed, 
        DoubleSupplier strafeSpeed, 
        DoubleSupplier turnSpeed, 
        BooleanSupplier halfSpeed, 
        BooleanSupplier headingCorrection,
        BooleanSupplier humanPlayerLeft,
        BooleanSupplier humanPlayerRight,
        BooleanSupplier snapHeading) {

        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.halfSpeed = halfSpeed;
        this.headingCorrection = headingCorrection;
        this.humanPlayerLeft = humanPlayerLeft;
        this.humanPlayerRight = humanPlayerRight;
        this.snapHeading = snapHeading;

        addRequirements(drive);
    }
    @Override
    public void initialize() {
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        var result = DriverStation.getAlliance();
        if (result.isPresent()) {
            alliance = result.get();
        }
    }

    @Override
    public void execute() {
        double speedMultiplier = halfSpeed.getAsBoolean() ? 0.5 : 1.0;

        var outputSpeeds = new ChassisSpeeds(
            xSlewRateLimiter.calculate(joystickToVelocity(forwardSpeed.getAsDouble() * speedMultiplier)),
            ySlewRateLimiter.calculate(joystickToVelocity(strafeSpeed.getAsDouble() * speedMultiplier)),
            Math.pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), controllerDeadband), 3) * DrivetrainConstants.maxAngularVelocity);
        
        SmartDashboard.putNumber("lastHeading", lastHeading);

        int totalDesiredActions = (headingCorrection.getAsBoolean() ? 1 : 0) + (humanPlayerLeft.getAsBoolean() ? 1 : 0) + (humanPlayerRight.getAsBoolean() ? 1 : 0) + (snapHeading.getAsBoolean() ? 1 : 0);

        if (totalDesiredActions != 1) {
            ;
        }
        else if (headingCorrection.getAsBoolean()) {
            if (Math.abs(outputSpeeds.omegaRadiansPerSecond) < DrivetrainConstants.headingCorrectionDeadband
                && (Math.abs(outputSpeeds.vxMetersPerSecond) > DrivetrainConstants.headingCorrectionDeadband 
                || Math.abs(outputSpeeds.vyMetersPerSecond) > DrivetrainConstants.headingCorrectionDeadband)) 
            {
                outputSpeeds.omegaRadiansPerSecond = headingController.calculate(drive.getRotation2d().getRadians(), lastHeading);
            } else {
                lastHeading = drive.getRotation2d().getRadians();
            }
        } else if (humanPlayerLeft.getAsBoolean()) {
            outputSpeeds.omegaRadiansPerSecond = headingController.calculate(drive.getRotation2d().getRadians(), DrivetrainConstants.humanPlayerLeft);

        } else if (humanPlayerRight.getAsBoolean()) {
            outputSpeeds.omegaRadiansPerSecond = headingController.calculate(drive.getRotation2d().getRadians(), DrivetrainConstants.humanPlayerRight);
        } else if (snapHeading.getAsBoolean()) {
            var currentHeading = drive.getRotation2d();
            var closestAngle = DrivetrainConstants.snapAngles[0];
            double closestDistance = Math.abs(currentHeading.minus(closestAngle).getRadians());
            for (var angle : DrivetrainConstants.snapAngles) {
                double distance = Math.abs(currentHeading.minus(angle).getRadians());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestAngle = angle;
                }
            }
            outputSpeeds.omegaRadiansPerSecond = headingController.calculate(currentHeading.getRadians(), closestAngle.getRadians());

        }

        if (alliance == Alliance.Blue) {
            drive.fieldOrientedDrive(outputSpeeds);
        } else {
            drive.fieldOrientedDrive(outputSpeeds, drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0)));
        }

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    private double joystickToVelocity(double n) {
        return Math.pow(MathUtil.applyDeadband(n, controllerDeadband), 3) * DrivetrainConstants.maxVelocity;
    }
    
}