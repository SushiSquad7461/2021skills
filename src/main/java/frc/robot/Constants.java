/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

	// drivetrain
	public static final class Drivetrain {

		// motors
		public static final int FL_ID = 21;
		public static final int FR_ID = 22;
		public static final int BR_ID = 23;
		public static final int BL_ID = 24;

		// encoders
		public static final int ENCODER_LEFT = 3;
		public static final int ENCODER_RIGHT = 4;

		// open loop configuration
		public static final int	CURRENT_LIMIT = 35;
		public static final int	OPEN_LOOP_RAMP = 0;
		public static final double SLOW_SPEED = 0.1;

		public static final double CONTROLLER_LINEAR_SCALING = 75;
		public static final double CONTROLLER_ANGULAR_SCALING = 100;
		public static final double CONTROLLER_QUICKTURN_SCALING = 400;

		// pid constants
		public static final double LEFT_kP = 0.356;
		public static final double LEFT_kI = 0;
		public static final double LEFT_kD = 0;

		public static final double LEFT_kS = 0.166;
		public static final double LEFT_kV = 0.0315;
		public static final double LEFT_kA = 0.00743;

		public static final double RIGHT_kP = 0.356;
		public static final double RIGHT_kI = 0;
		public static final double RIGHT_kD = 0;

		public static final double RIGHT_kS = 0.166;
		public static final double RIGHT_kV = 0.0315;
		public static final double RIGHT_kA = 0.00743;
		
		// closed loop constants
		public static final double wheelRadius = 3.0;
		public static final double trackWidth = 0.6420;

		public static final double encoderResolution = 0;

	}

	// flywheel
	public static final class Flywheel {

		// motors
		public static final int MAIN_ID = 11;
		public static final int SECONDARY_ID = 10;

		public static final boolean MAIN_INVERTED = false;
		public static final boolean SECONDARY_INVERTED = true;

		// encoders
		public static final int ENCODER = 60;

		// flywheel speed (rotations per minute)
		public static final double SPEED = 5100.7461;

		// don't tune these--documentation stated constants for entering rpm
		public static final int TICKS_PER_ROTATION = 4096; // ticks per one encoder rotation
		public static final double SETPOINT_CONSTANT = 0.001667; // 100 ms / 1 min

		// pid constants
		public static final double kP = 0.0002;
		public static final double kI = 0.000000;
		public static final double kD = 0;

		public static final double kS = 0.633;
		public static final double kV = 0.00201;
		public static final double kA = 0.000405;
		public static final double ERROR_TOLERANCE = 0;
		public static final double SPEED_TOLERANCE = 100;

		// timeout value for parameter configs
		public static final int CONFIG_TIMEOUT = 30;

	}

	// hopper
	public static final class Hopper {
		public static final int FAST_ID = 1;
		public static final int SLOW_ID = 6;

		public static final double MAX_SPEED = -0.6;
		public static final double SLOW_SPEED = -0.2;
		public static final double REVERSE_SPEED = 0.6;

		public static final int CONFIG_TIMEOUT = 30;
		public static final int CURRENT_SPIKE = 70;
	}

  // intake
	public static final class Intake {
		public static final int MOTOR_ID = 2;
		public static final double MAX_SPEED = -0.65;
	}
  
	// camera
	public static final class Camera {

	}
	// climb
	public static final class ClimbConstants {
		public static final int DEPLOY_MOTOR_ID = 31;
		public static final int FOLLOWER_MOTOR_ID = 32;

		// climb motor inverted
		public static final boolean TALON_INVERTED = true;

		// climb encoder CAN ID
		public static final int CLIMB_CAN_ID = 42;

		// climb PID constants (need to tune)
		public static final int ARM_kP = 0;
		public static final int ARM_kI = 0;
		public static final int ARM_kD = 0;

		// climb motion profiling constants (need to tune)
		public static final int MAX_VELOCITY_RAD_PER_SEC = 69;
		public static final int MAX_ACCEL = 69;

		// Encoder distance per pulse
		public static final int PULSES_PER_ROTATION = 4096;
		public static final double DISTANCE_PER_PULSE = (2.0 * Math.PI) / PULSES_PER_ROTATION;

		// starting arm position (need to set)
		public static final double BASE_POSE = 0;

		// feedforward constants; kS = volts; kG = volts (counteracts gravity); kV = volts * sec / radians; kA = volts * sec^2 / radians
		// (need to characterize)
		public static final double kS = 0.0;
		public static final double kG = 0.0;
		public static final double kV = 0.0;
		public static final double kA = 0.0;

		// elevator linear speed in inches (please tell me when we know these things)
		public static final double CLIMB_ELEVATOR_DISTANCE_PER_ROTATION = 0;

		public static final double CLIMB_SPEED = -1.0;
		public static final double CLIMB_SLOW_SPEED = -0.2;
		public static final double CLIMB_STALL_SPEED = 0.15;

		// climb heights
		public static final double MAX_SPRING_HEIGHT = 10;

	}

	public static final class Ramsete {
		public static final double RAMSETE_B = 2;
		public static final double RAMSETE_ZETA = 0.7;
		public static final double kP_VEL_LEFT = 0.001;
		public static final double kP_VEL_RIGHT = 0.001;
		public static final double MAX_VOLTAGE_CONSTRAINT = 40;
		public static final double kS = 0.166;
		public static final double kV = 0.0315;
		public static final double kA = 0.00743;
		public static final double MAX_METERS_PER_SECOND = 5.45592;
		public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 4.0;
	}

}
