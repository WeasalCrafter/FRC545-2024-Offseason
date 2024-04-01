// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
	public static final String kDrivingTabName = "Driver Station";
	public static final String kDebugTabName = "Debug";

	public static final double ROBOT_MASS = (90) * 0.453592; // 90lbs * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS); //TODO
	public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

	public static final class CANConstants {
		public static final int INTAKE_MOTOR_1 = 11;
		public static final int INTAKE_MOTOR_2 = 12;

		public static final int SHOOTER_MOTOR_1 = 13;
		public static final int SHOOTER_MOTOR_2 = 14;
		
		public static final int SUPPORT_MOTOR_1 = 15;
		public static final int SUPPORT_MOTOR_2 = 16;

		public static final int ACTUATOR_MOTOR_1 = 17;
		public static final int ACTUATOR_MOTOR_2 = 18;
	}

	public static final class AutonConstants {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
		public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
	}

	public static final class DrivebaseConstants {
		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static class OperatorConstants {
		// Joystick Deadband
		public static final double LEFT_X_DEADBAND = 0.1;
		public static final double LEFT_Y_DEADBAND = 0.1;
		public static final double RIGHT_X_DEADBAND = 0.1;
		public static final double TURN_CONSTANT = 6;
	}

	public static class SpeedConstants {
		public static final double SpeakerShooterSpeed = 1.00;
		public static final double AmpShooterSpeed = 0.35;
		public static final double HumanIntakeSpeed = 0.20;
		public static final double IntakeSpeed = 0.50;

		public static final double SpeakerOutakeDelay = 0.1;
		public static final double SpeakerShooterDelay = 1;

		public static final double ClimberSpeed = 0.5;
	}
}
