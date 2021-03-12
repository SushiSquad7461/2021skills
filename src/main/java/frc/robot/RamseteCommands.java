package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RamseteCommands {
	private final RobotContainer m_container;
	private final Paths path;
	private DifferentialDriveVoltageConstraint voltageConstraint;
	private RamseteCommand exampleCommand;

	public RamseteCommands(RobotContainer container) {
		m_container = container;

		voltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.Ramsete.kS,
						Constants.Ramsete.kV, Constants.Ramsete.kA),
				m_container.s_drive.driveKinematics,
				Constants.Ramsete.MAX_VOLTAGE_CONSTRAINT
		);

		path = new Paths(container, voltageConstraint);

		//exampleCommand = defineRamseteCommand(path.example);
	}
	/*
	public RamseteCommand defineRamseteCommand(Trajectory traj) {
		return new RamseteCommand(
				traj,
				m_container.s_drive::getPose,
				new RamseteController(Constants.Ramsete.RAMSETE_B, Constants.Ramsete.RAMSETE_ZETA),
				new SimpleMotorFeedforward(Constants.Ramsete.kS, Constants.Ramsete.kV, Constants.Ramsete.kA),
				m_container.s_drive.driveKinematics,
				m_container.s_drive::getWheelSpeeds,
				new PIDController(Constants.Ramsete.kP_VEL_LEFT, 0, 0),
				new PIDController(Constants.Ramsete.kP_VEL_RIGHT, 0, 0);
				m_container.s_drive::tankDriveVolts,
				m_container.s_drive
		);
	} */
}
