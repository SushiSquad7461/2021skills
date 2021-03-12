/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.chassis;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

	// define variables
	private CANSparkMax frontLeft, frontRight, backLeft, backRight;
	//private CANCoder leftEncoder, rightEncoder;
	private AHRS nav;

	private boolean driveInverted = false;
	private boolean slowMode = false;
	private DifferentialDrive differentialDrive;
	public DifferentialDriveKinematics driveKinematics;
	private DifferentialDriveOdometry driveOdometry;
	private SimpleMotorFeedforward leftFeedforward, rightFeedforward;
	private PIDController leftController, rightController;

	public Drivetrain() {

		// configuration
		CANSparkMaxLowLevel.MotorType brushless = CANSparkMaxLowLevel.MotorType.kBrushless;

		double wheelRadius = Constants.Drivetrain.wheelRadius;
		double encoderResolution = Constants.Drivetrain.encoderResolution;

		// instantiate objects

		frontLeft = new CANSparkMax(Constants.Drivetrain.FL_ID, brushless);
		frontRight = new CANSparkMax(Constants.Drivetrain.FR_ID, brushless);
		backLeft = new CANSparkMax(Constants.Drivetrain.BL_ID, brushless);
		backRight = new CANSparkMax(Constants.Drivetrain.BR_ID, brushless);

		//leftEncoder = new CANCoder(Constants.Drivetrain.ENCODER_LEFT);
		//rightEncoder = new CANCoder(Constants.Drivetrain.ENCODER_RIGHT);

		resetEncoders();
		nav = new AHRS(SPI.Port.kMXP);
		nav.reset();

		differentialDrive = new DifferentialDrive(frontLeft, frontRight);
		driveKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.trackWidth);
		driveOdometry = new DifferentialDriveOdometry(getAngle());

		leftFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.LEFT_kS, Constants.Drivetrain.LEFT_kV, Constants.Drivetrain.LEFT_kA);
		rightFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.RIGHT_kS, Constants.Drivetrain.RIGHT_kV, Constants.Drivetrain.RIGHT_kA);

		leftController = new PIDController(Constants.Drivetrain.LEFT_kP, Constants.Drivetrain.LEFT_kI, Constants.Drivetrain.LEFT_kD);
		rightController = new PIDController(Constants.Drivetrain.RIGHT_kP, Constants.Drivetrain.RIGHT_kI, Constants.Drivetrain.RIGHT_kD);

		// configure motor controllers
		backLeft.follow(frontLeft);
		backRight.follow(frontRight);

		// open loop inversion configuration
		frontLeft.setInverted(driveInverted);
		frontRight.setInverted(driveInverted);
		backLeft.setInverted(driveInverted);
		backRight.setInverted(driveInverted);

		// closed loop inversion configuration
		/*frontLeft.setInverted(driveInverted);
		frontRight.setInverted(!driveInverted);
		backLeft.setInverted(driveInverted);
		backRight.setInverted(!driveInverted);*/

		frontLeft.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP);
		frontRight.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP);
		backLeft.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP);
		backRight.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP);

		frontLeft.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
		frontRight.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
		backLeft.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
		backRight.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);

	}

	// open loop curve drive method
	public void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickTurn) {
		if (slowMode) {
			differentialDrive.curvatureDrive(linearVelocity * Constants.Drivetrain.SLOW_SPEED, angularVelocity, isQuickTurn);
		} else {
			SmartDashboard.putNumber("auto velo", 5);
			differentialDrive.curvatureDrive(linearVelocity, angularVelocity, isQuickTurn);
		}
	}

	// closed loop drive method
	public void closedCurveDrive(double linearVelocity, double angularVelocity, boolean isQuickTurn) {

		ChassisSpeeds chassisSpeeds;
		DifferentialDriveWheelSpeeds wheelSpeeds;
		double leftOutput, rightOutput;
		double leftFeedforwardOutput, rightFeedforwardOutput;

		if (!isQuickTurn && linearVelocity > 0.02) {
			chassisSpeeds = new ChassisSpeeds(linearVelocity * Constants.Drivetrain.CONTROLLER_LINEAR_SCALING, 0, angularVelocity * Constants.Drivetrain.CONTROLLER_ANGULAR_SCALING);
		} else {
			chassisSpeeds = new ChassisSpeeds(0, 0, angularVelocity * Constants.Drivetrain.CONTROLLER_QUICKTURN_SCALING);
		}

		wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

		leftFeedforwardOutput = leftFeedforward.calculate(wheelSpeeds.leftMetersPerSecond);
		rightFeedforwardOutput = rightFeedforward.calculate(wheelSpeeds.rightMetersPerSecond);

		//leftOutput = leftController.calculate(leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
		//rightOutput = rightController.calculate(rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);

		//frontLeft.setVoltage(leftOutput + leftFeedforwardOutput);
		//frontRight.setVoltage(rightOutput + rightFeedforwardOutput);

	}

	// toggle slowmode on open loop drive
	public void toggleSlow() {
		slowMode = !slowMode;
	}

	// get angle from gyro
	public Rotation2d getAngle() {
		return Rotation2d.fromDegrees(nav.getAngle());
	}

	// zero the navx
	public void zeroAngle() {
		nav.reset();
	}

	// reset the encoders
	public void resetEncoders() {
		//leftEncoder.setPosition(0);
		//rightEncoder.setPosition(0);
	}

	// update drive odometry
	public void updateOdometry() {
		//driveOdometry.update(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());
	}

	// reset odometry
	public void resetOdometry() {
		resetEncoders();
		driveOdometry.resetPosition(getPose(), getAngle());
	}

	// get pose from odometry
	public Pose2d getPose() {
		return driveOdometry.getPoseMeters();
	}

	// return velocities of respective drive sides
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		//return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
		return null;
	}

	// tank drive with voltage input
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		frontLeft.setVoltage(leftVolts);
		frontRight.setVoltage(-rightVolts);
	}

	@Override
	public void periodic() {
		updateOdometry();
	}

}
