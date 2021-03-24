/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel;
public class Hood extends SubsystemBase {

  
  CANSparkMaxLowLevel.MotorType brushless = CANSparkMaxLowLevel.MotorType.kBrushless;
  private CANSparkMax hoodMain;
  private CANEncoder hoodEncoder;

  public Hood() {

      // instantiate the neo550 and encoder
      hoodMain = new CANSparkMax(Constants.Hood.HOOD_MOTOR_ID, brushless);
      hoodEncoder = new CANEncoder(hoodMain);
  }

  // raises the hood, lowers angle of shot
  public void raiseHood() {
      hoodMain.set(Constants.Hood.HOOD_MAX_SPEED);
  }

  // lowers hood, increases angle of shot
  public void lowerHood() {
      hoodMain.set(-Constants.Hood.HOOD_MAX_SPEED);
  }


  protected void useOutput() {}

  protected double getMeasurement() { 
      return 0.0;
      //CHANGE
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
