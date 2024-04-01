// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

import java.io.File;
import java.io.IOException;
import java.util.Map;

import swervelib.parser.SwerveParser;

public class Robot extends TimedRobot {
		
	private static Robot instance;
	private Command m_autonomousCommand;
	private Command m_warmupCommand;

	private RobotContainer m_robotContainer;
	private Timer disabledTimer;
	private final String drivingTabName = Constants.kDrivingTabName;
	private final String debugTabName = Constants.kDebugTabName;
	private final Field2d m_field = new Field2d();
	private Drivetrain m_drivetrain;
	
	public Robot() {
		instance = this;
	}

	public static Robot getInstance() {
		return instance;
	}

	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
		m_drivetrain = m_robotContainer.getDrivetrain();
		disabledTimer = new Timer();

		m_warmupCommand = m_drivetrain.getAutonomousCommand("warmup");
		m_warmupCommand.schedule();
		
		updateSmartDashboard();
		setupShuffleboard();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		updateSmartDashboard();
	}

	@Override
	public void disabledInit() {
		m_robotContainer.setMotorBrake(true);
		disabledTimer.reset();
		disabledTimer.start();
	}

	@Override
	public void disabledPeriodic() {
		if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
		{
			m_robotContainer.setMotorBrake(false);
			disabledTimer.stop();
		}
	}

	@Override
	public void autonomousInit() {
		m_robotContainer.setMotorBrake(true);
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		m_robotContainer.setMotorBrake(true);
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		try {
			new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {}

	public void setupShuffleboard() {
		Shuffleboard.getTab(drivingTabName)
			.add("Camera Output", 1)
			.withWidget(BuiltInWidgets.kCameraStream)
			.withSize(4,4)
			.withProperties(Map.of("Show Controls", false, "Show Crosshair", false))
			.getEntry();
		
		Shuffleboard.getTab(drivingTabName)
			.add("Autonomous Chooser", SmartDashboard.getData("Autonomous Picker"))
			.withWidget(BuiltInWidgets.kComboBoxChooser)
			.withSize(4, 1)
			.withPosition(4, 2);
		
		Shuffleboard.getTab(drivingTabName)
			.add("FMS Info", 1)
			.withWidget("FMSInfo")
			.withSize(4, 1)
			.withPosition(4, 3)
			.getEntry();

		Shuffleboard.getTab(drivingTabName)
			.add("Field2D", m_field)
			.withWidget(BuiltInWidgets.kField)
			.withSize(5, 4)
			.withPosition(8, 0)
			.withProperties(Map.of("robot_width", 0.762, "robot_length", 0.762));

		Shuffleboard.getTab(drivingTabName)
			.add("Command Scheduler", CommandScheduler.getInstance())
			.withWidget("Scheduler")
			.withSize(2, 4)
			.withPosition(13, 0);

		Shuffleboard.getTab(debugTabName)
			.add("PDH", SmartDashboard.getData("Power Distribution Hub"))
			.withWidget("PowerDistribution")
			.withPosition(0, 0);

		// Shuffleboard.getTab(drivingTabName)
		// 	.add("Swerve Drive", SmartDashboard.getData("swerve"))
		// 	.withWidget("YAGSL Swerve Drive")
		// 	.withPosition(4, 0)
		// 	.withSize(2,2);
	}

	public void ElasticGraph(String graphName){
		Shuffleboard.getTab(drivingTabName)
			.add(graphName, SmartDashboard.getData(graphName))
			.withWidget(graphName)
			.withPosition(0, 2)
			.withSize(4, 2);
	}

	public void updateSmartDashboard() {
		m_field.setRobotPose(m_drivetrain.getPose());

	}
}
