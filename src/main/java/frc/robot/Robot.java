// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystemPos;
// import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final ElevatorSubsystemPos m_Elevator = new ElevatorSubsystemPos();
    private final ArmSubsystem m_Arm = new ArmSubsystem();
    // private final IntakeSubsystem m_Intake = new IntakeSubsystem();
    private RobotContainer m_robotContainer;
    


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_Arm.config();
        m_Arm.resetEncoders();
        m_Elevator.config();
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture();
    }


    @Override
    public void robotPeriodic() {
        // Read values from the encoders regularly.
        m_Arm.ArmPeriodic();
        m_Elevator.periodic();

        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        m_robotContainer.setAuto(true);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        double driveSpeed = 0.1;
        m_robotContainer.driveForwards(driveSpeed);
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.setAuto(false);
        m_Elevator.configureElevatorBindings();
        // m_Intake.configureBindings();
        m_Arm.configureArmBindings();
        // ElevatorSubsystem.elevatorMotor.configure(ElevatorSubsystem.config, null, null);
    }

    /** This function is called periodically during operator controwl. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
