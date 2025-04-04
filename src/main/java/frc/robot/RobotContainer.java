// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystemPos;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final XboxController mController = new XboxController(Constants.kControllerPort);
    private final Swerve mSwerve = new Swerve();

    // Create new instance of ManualDrive, passing Swerve and Controller as parameters
    private final ManualDrive mManualDriveCommand = new ManualDrive(mSwerve, mController);

    // Extract trajectory from PathPlanner
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        mSwerve.setDefaultCommand(mManualDriveCommand);
    }

    private void configureButtonBindings() {}

    public void driveForwards(double speed) {
        mManualDriveCommand.drive(-1*speed);
    }

    public void setAuto(boolean setAuto){
        mManualDriveCommand.setAuto(setAuto);
    }
}
