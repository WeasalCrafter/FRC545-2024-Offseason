// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Climbers;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
	private final Drivetrain m_drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(), "swerve"));
	private final IntakeShoot m_intakeShoot = new IntakeShoot();
	private final Climbers m_climbers = new Climbers();

	private final CommandXboxController m_driverController = new CommandXboxController(0);
	private final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);	
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		registerCommands();
		setDefaultCommands();
		configureBindings();	

		autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser.setDefaultOption("Warmup", m_drivetrain.getAutonomousCommand("warmup"));
		SmartDashboard.putData("Autonomous Picker", autoChooser);
		SmartDashboard.putData("Power Distribution Hub", PDH);
	}
	
	private void configureBindings() {

		m_driverController.leftBumper().and(m_driverController.rightBumper()).onTrue((Commands.runOnce(m_drivetrain::zeroGyro)));

		m_driverController.x().whileTrue(Commands.runOnce(m_drivetrain::lock, m_drivetrain).repeatedly());

		m_driverController.rightTrigger() // AMP SHOT
			.whileTrue(m_intakeShoot.startAmpShot())
			.onFalse(m_intakeShoot.stopShooter());
		
		m_driverController.leftTrigger() // GROUND INTAKE
			.whileTrue(m_intakeShoot.startIntake())
			.onFalse(m_intakeShoot.stopIntake());

		m_driverController.rightBumper() // SOURCE INTAKE
			.whileTrue(m_intakeShoot.startHumanIntake())
			.onFalse(m_intakeShoot.stopShooter());

		m_driverController.leftBumper() // OUTTAKE
			.whileTrue(m_intakeShoot.startOutake())
			.onFalse(m_intakeShoot.stopIntake());
		
		m_driverController.rightTrigger() // HIGH SHOT
			.and(m_driverController.rightStick())
				.whileTrue(m_intakeShoot.startSpeakerShot())
				.onFalse(m_intakeShoot.stopShooter());

		m_driverController.pov(0) // UP
			.whileTrue(m_climbers.startClimbers())
			.onFalse(m_climbers.stopClimbers());

		m_driverController.pov(180) // DOWN
			.whileTrue(m_climbers.reverseClimbers())
			.onFalse(m_climbers.stopClimbers());
	}
	
	public void registerCommands(){
		NamedCommands.registerCommand("Start Intake", m_intakeShoot.startIntake());
		NamedCommands.registerCommand("Start Outake", m_intakeShoot.startOutake());
		NamedCommands.registerCommand("Stop Intake", m_intakeShoot.stopIntake());

		NamedCommands.registerCommand("Start Speaker Shot", m_intakeShoot.startSpeakerShot());
		NamedCommands.registerCommand("Start Human Intake", m_intakeShoot.startHumanIntake());
		NamedCommands.registerCommand("Start Amp Shot", m_intakeShoot.startAmpShot());
		NamedCommands.registerCommand("Stop Shooter", m_intakeShoot.stopShooter());

		NamedCommands.registerCommand("Temp Amp Shot", m_intakeShoot.startAmpShot());
		NamedCommands.registerCommand("Full Speaker Shot", m_intakeShoot.fullSpeakerShot());

	}

	public void setDefaultCommands() {
		// right stick controls the angular velocity of the robot
		Command driveFieldOrientedAngularVelocity = m_drivetrain.driveCommand(
			() -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
			() -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
			() -> m_driverController.getRightX() * 0.5
		);
		
		Command driveFieldOrientedAngularVelocitySim = m_drivetrain.simDriveOmegaCommand(
			() -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
			() -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
			() -> m_driverController.getRightX()
		);

		m_drivetrain.setDefaultCommand(
			!RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedAngularVelocitySim
		);	

	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setMotorBrake(boolean brake){
		m_drivetrain.setMotorBrake(brake);
	}

	public Drivetrain getDrivetrain() {
		return m_drivetrain;
	}
}
