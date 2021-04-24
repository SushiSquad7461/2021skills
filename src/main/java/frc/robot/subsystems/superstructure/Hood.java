/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.*;

public class Hood extends SubsystemBase {
    private CANSparkMax hoodMain;
    private CANEncoder hoodEncoder;
    private CANPIDController hoodController;
    private InterpolatingTreeMap closeTreeMap;
    private InterpolatingTreeMap farTreeMap;
    private PhotonCamera camera;

    public Hood() {
    
        SmartDashboard.putNumber("kP", Constants.Hood.kP);
        SmartDashboard.putNumber("kD", Constants.Hood.kD);
        this.hoodMain = new CANSparkMax(Constants.Hood.MOTOR_ID, Constants.Hood.MOTOR_TYPE);
        hoodMain.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.hoodEncoder = this.hoodMain.getEncoder();
        this.hoodController = this.hoodMain.getPIDController();
        this.hoodController.setOutputRange(-Constants.Hood.MAX_SPEED, Constants.Hood.MAX_SPEED);
        this.hoodController.setP(Constants.Hood.kP);
        this.hoodController.setI(Constants.Hood.kI);
        this.hoodController.setD(Constants.Hood.kD);
        this.zeroHood();
        this.camera = new PhotonCamera("myCamera");
        //this.setSetpoint(50);
        
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("real output", hoodMain.getAppliedOutput());
        SmartDashboard.putNumber("bruh position", hoodEncoder.getPosition());
    }

    public void zeroHood() {
        this.hoodEncoder.setPosition(Constants.Hood.INITIAL_SETPOIONT);
    }

    public void setSetpoint(double setpoint) {
        this.hoodController.setReference(setpoint, ControlType.kPosition);
    }
}
