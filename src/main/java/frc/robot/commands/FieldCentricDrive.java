package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class FieldCentricDrive extends Command {
    private final SwerveDrivetrain drive;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier turnSpeed;
    private final BooleanSupplier halfSpeed;
    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(10.0);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(10.0);
    private SlewRateLimiter wSlewRateLimiter = new SlewRateLimiter(10.0);
    private static double controllerDeadband = 0.05;
    /**
     * Teleop drive command
     * @param drive swerve drivetrain
     * @param forwardSpeed -1.0 to 1.0
     * @param strafeSpeed -1.0 to 1.0
     * @param turnSpeed -1.0 to 1.0
     * @param halfSpeed doesn't actually work
     */
    public FieldCentricDrive(SwerveDrivetrain drive, DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed, BooleanSupplier halfSpeed) {
        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.halfSpeed = halfSpeed;

        addRequirements(drive);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speedMultiplier = halfSpeed.getAsBoolean() ? 0.1 : 1.0;

        var outputSpeeds = new ChassisSpeeds(
            xSlewRateLimiter.calculate(joystickToVelocity(forwardSpeed.getAsDouble() * speedMultiplier)),
            ySlewRateLimiter.calculate(joystickToVelocity(strafeSpeed.getAsDouble() * speedMultiplier)),
            wSlewRateLimiter.calculate(joystickToVelocity(turnSpeed.getAsDouble() * speedMultiplier)));

        drive.fieldOrientedDrive(outputSpeeds);

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    private double joystickToVelocity(double n) {
        return Math.pow(MathUtil.applyDeadband(strafeSpeed.getAsDouble(), controllerDeadband), 3) * DrivetrainConstants.maxVelocity;
    }
    
}