package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class ManualDrive extends Command {
    private final Swerve mSwerve;
    private final XboxController mController;
    
    public ManualDrive(Swerve drive, XboxController controller) {
        mSwerve = drive;
        mController = controller;

        // Adds the Swerve subsystem as a requirement to the command
        addRequirements(mSwerve);
    }

    @Override
    public void execute() {
        // Drives with XSpeed, YSpeed, zSpeed
        // True/false for field-oriented driving
        mSwerve.drive(-MathUtil.applyDeadband(mController.getLeftY(), 0.05),
                -MathUtil.applyDeadband(mController.getLeftX(), 0.05),
                -MathUtil.applyDeadband(mController.getRightX(), 0.05),false);
    }
}
