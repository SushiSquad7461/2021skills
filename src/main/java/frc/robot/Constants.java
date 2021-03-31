/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public final class Constants {

    // drivetrain
    public static final class Drivetrain {

        // motors
        public static final int FL_ID = 18;
        public static final int FR_ID = 15;
        public static final int BR_ID = 14;
        public static final int BL_ID = 1;
        public static final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;

        // open loop configuration
        public static final int CURRENT_LIMIT = 35;
        public static final int OPEN_LOOP_RAMP = 0;
        public static final double SLOW_SPEED = 0.1;

    }

    // flywheel
    public static final class Flywheel {

        // motors
        public static final int MAIN_ID = 13;
        public static final int SECONDARY_ID = 3;
        public static final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;

        public static final boolean MAIN_INVERTED = true;
        public static final boolean SECONDARY_INVERTED = false;

        // encoders
        public static final int ENCODER = 60;

        // flywheel speed (rotations per minute)
        public static final double SPEED = 5100.7461;

        // don't tune these--documentation stated constants for entering rpm
        public static final int TICKS_PER_ROTATION = 42; // ticks per one encoder rotation
        public static final double SETPOINT_CONSTANT = 0.001667; // 100 ms / 1 min

        // pid constants
        public static final double kP = 1.320007461;
        public static final double kI = 0.000000;
        public static final double kD = 0;

        public static final double kS = 0.529;
        public static final double kV = 0.136;
        public static final double kA = 0.0306;
        public static final double ERROR_TOLERANCE = 0;
        public static final double SPEED_TOLERANCE = 100;

        // timeout value for parameter configs
        public static final int CONFIG_TIMEOUT = 30;

        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> rpmTreeMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
    }

    // hopper
    public static final class Hopper {
        public static final int FAST_ID = 11;
        public static final int SLOW_ID = 10;
        public static final int FLOOR_ID = 9;
        public static final double MAX_SPEED = -0.6;
        public static final double SLOW_SPEED = -0.2;
        public static final double REVERSE_SPEED = 0.6;
        public static final double FLOOR_SPEED = 1.0;
        public static final int KICKER_ID = 2;
        public static final int CONFIG_TIMEOUT = 30;
        public static final int CURRENT_SPIKE = 70;
    }

    // intake
    public static final class Intake {
        public static final int MOTOR_ID = 2;
        public static final double MAX_SPEED = -0.65;

        // solenoid
        public static final int SOLENOID_IN = 0;
        public static final int SOLENOID_OUT = 1;
    }

    public static final class Hood {
        public static final int MOTOR_ID = 12;
        public static final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
        public static final double MAX_SPEED = 0.3;
        public static final double kP = 0.25;
        public static final double kI = 0.0;
        public static final double kD = 0.00;
        public static final double SETPOINT = (45.0/360.0) * (332.0/14.0) * 15;
        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> angleTreeMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

    }
    // camera
    public static final class Camera {
        public static final double CAMERA_HEIGHT_METERS = 0;
        public static final double TARGET_HEIGHT_METERS = 0;
        public static final double CAMERA_PITCH_RADIANS = 0;
    }

    // OI
    public static final class OI {
        public static final int DRIVE_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;

		public static final double TRIGGER_TOLERANCE = 0.3;
	}
    
    public static final class Vision {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double THRESHOLD = 0.1;
    }

}
