/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.Flywheel;

public class Shoot extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Flywheel s_flywheel;

	public Shoot(Flywheel flywheel) {
		s_flywheel = flywheel;
		addRequirements(flywheel);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {

		if (OI.getTriggerBoolean(RobotContainer.operatorController, GenericHID.Hand.kRight)) {
			s_flywheel.enableFlywheel();
		} else {
			s_flywheel.stop();
		}

	}

	@Override
	public void end(boolean interrupted) {
		s_flywheel.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
