// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.commands.FollowPathCommand;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        // URCL.start(Map.of(1, "fl", 2, "fr", 3, "bl", 4, "br"));

        CanBridge.runTCP();

        FollowPathCommand.warmupCommand().schedule();
    }


    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.

        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_robotContainer.drive.removeDefaultCommand();
        m_robotContainer.elevator.removeDefaultCommand();
        m_robotContainer.arm.removeDefaultCommand();
        m_robotContainer.drive.driverMode(false);
        m_robotContainer.climber.removeDefaultCommand();
        
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        m_robotContainer.drive.driverMode(true);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.drive.setDefaultCommand(m_robotContainer.defaultDriveCommand());
        m_robotContainer.elevator.setDefaultCommand(m_robotContainer.defaultElevatorCommand());
        m_robotContainer.arm.setDefaultCommand(m_robotContainer.defaultArmCommand());
        m_robotContainer.climber.setDefaultCommand(m_robotContainer.defaultClimberCommand());
        // m_robotContainer.manipulator.setIntake(0.0);
        // m_robotContainer.manipulator.setDefaultCommand(m_robotContainer.defaultManipulatorCommand());
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {

    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
