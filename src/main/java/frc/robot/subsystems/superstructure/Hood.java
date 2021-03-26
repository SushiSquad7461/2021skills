/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel;

public class Hood extends SubsystemBase {

    // fields
    private CANSparkMax hoodMain; // main motor
    private CANEncoder hoodEncoder; // main motor encoder

    public Hood() {
        // instantiate neo550 and encoder
        hoodMain = new CANSparkMax(Constants.Hood.HOOD_MOTOR, Constants.Hood.MOTOR_TYPE);
        hoodEncoder = hoodMain.getEncoder();
    }

    // raises the hood, lowers angle of shot
    public void raiseHood() {
        hoodMain.set(Constants.Hood.HOOD_MAX_SPEED);
    }

    // lowers hood, increases angle of shot
    public void lowerHood() {
        hoodMain.set(-Constants.Hood.HOOD_MAX_SPEED);
    }

    // stop hood movement
    public void stopHood() { hoodMain.set(0); }

    protected void useOutput() {
    }

    protected double getMeasurement() {
        return 0.0;
        //CHANGE
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
