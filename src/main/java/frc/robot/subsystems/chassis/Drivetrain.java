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

    // fields
    private CANSparkMax frontLeft, frontRight, backLeft, backRight; // drive motors
    private AHRS nav; // gyro

    private DifferentialDrive differentialDrive;
    private boolean driveInverted; // false for normal forward direction, true when inverted
    private boolean slowMode; // true to reduce drive speed
    public DifferentialDriveKinematics driveKinematics; // calculates kinematics

    public Drivetrain() {

        // define fields
        driveInverted = false;
        slowMode = false;

        // instantiate/configure motor controllers
        frontLeft = new CANSparkMax(Constants.Drivetrain.FL_ID, Constants.Drivetrain.MOTOR_TYPE);
        frontRight = new CANSparkMax(Constants.Drivetrain.FR_ID, Constants.Drivetrain.MOTOR_TYPE);
        backLeft = new CANSparkMax(Constants.Drivetrain.BL_ID, Constants.Drivetrain.MOTOR_TYPE);
        backRight = new CANSparkMax(Constants.Drivetrain.BR_ID, Constants.Drivetrain.MOTOR_TYPE);
        differentialDrive = new DifferentialDrive(frontLeft, frontRight);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        frontLeft.setInverted(driveInverted);
        frontRight.setInverted(driveInverted);
        backLeft.setInverted(driveInverted);
        backRight.setInverted(driveInverted);

        frontLeft.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP);
        frontRight.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP);
        backLeft.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP);
        backRight.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP);

        frontLeft.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
        frontRight.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
        backLeft.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
        backRight.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);

        // configure gyro
        nav = new AHRS(SPI.Port.kMXP);
        nav.reset();
    }

    // open loop curve drive method
    public void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickTurn) {
        if (slowMode) {
            differentialDrive.curvatureDrive(linearVelocity * Constants.Drivetrain.SLOW_SPEED,
                    angularVelocity, isQuickTurn);
        } else {
            differentialDrive.curvatureDrive(linearVelocity, angularVelocity, isQuickTurn);
        }
    }

    // toggle slowmode on open loop drive
    public void toggleSlow() {
        slowMode = !slowMode;
    }

    @Override
    public void periodic() {
    }

}
