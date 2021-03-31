/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.superstructure.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.chassis.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Hopper;
import frc.robot.subsystems.superstructure.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

	// initialize subsystems
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    public final Drivetrain s_drive;
    public final Flywheel s_flywheel;
    private final Hopper s_hopper;
    public final Intake s_intake;
    public final Hood s_hood;

    // initialize commands
    public final Shoot c_shoot;
    public final AutoShoot c_autoShoot;
    public final AutoDrive c_autoDrive;

    // create joysticks
    public static final XboxController driveController = new XboxController(Constants.OI.DRIVE_CONTROLLER);
    public static final XboxController operatorController = new XboxController(Constants.OI.OPERATOR_CONTROLLER);

    public RobotContainer() {
        // subsystems
        s_drive = new Drivetrain();
        s_flywheel = new Flywheel();
        s_hopper = new Hopper();
        s_intake = new Intake();
        s_hood = new Hood();

        // commands
        c_shoot = new Shoot(s_flywheel);
        c_autoShoot = new AutoShoot(s_flywheel, s_hopper);
        c_autoDrive = new AutoDrive(s_drive);

        // set default commands
        s_flywheel.setDefaultCommand(c_shoot);


        s_drive.setDefaultCommand(new RunCommand(
            () -> s_drive.curveDrive(
                OI.getTriggerOutput(driveController),
                OI.getLeftJoystickAxis(driveController),
                driveController.getXButton()), s_drive));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // run hopper
        new JoystickButton(operatorController, XboxController.Button.kA.value)
            .whenPressed(new RunCommand(s_hopper::startSpit, s_hopper))
            .whenReleased(new RunCommand(s_hopper::endSpit, s_hopper));

        // intake
        new JoystickButton(operatorController, XboxController.Button.kA.value)
            .whenPressed(new RunCommand(s_intake::startVore, s_intake))
            .whenReleased(new RunCommand(s_intake::stopVore, s_intake));

        // intake unjam
        new JoystickButton(operatorController, XboxController.Button.kY.value)
            .whenPressed(new RunCommand(s_intake::unVore, s_intake))
            .whenReleased(new RunCommand(s_intake::stopVore, s_intake));
    }

    public SequentialCommandGroup getAutonomousCommand() {
        return new SequentialCommandGroup();
    }

}
