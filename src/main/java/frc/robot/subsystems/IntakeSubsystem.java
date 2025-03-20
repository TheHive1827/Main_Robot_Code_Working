package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix.motorcontrol.IFollower;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {
  static CommandXboxController ElevatorController = new CommandXboxController(1);
  XboxController exampleXbox = new XboxController(1); // 0 is the USB Port to be used as indicated on the Driver Station
  private final static SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeID, MotorType.kBrushless);
  boolean LeftBump = exampleXbox.getLeftBumperButtonPressed();
  boolean RightBump = exampleXbox.getRightBumperButtonPressed();
  boolean ButtonX = exampleXbox.getXButtonPressed();
  

  public void configureBindings() {
    LeftBump = exampleXbox.getLeftBumperButtonPressed();
    RightBump = exampleXbox.getRightBumperButtonPressed();
      if (LeftBump == false && RightBump == false){
        ElevatorController.leftBumper().whileFalse(IntakeCommand(0));
        ElevatorController.rightBumper().whileFalse(IntakeCommand(0));
        // when both of them are NOT being pressed, when either of them are NOT ebing pressed it stops the motors.
      }
       else {
        ElevatorController.leftBumper().whileTrue(IntakeCommand(1));
        ElevatorController.rightBumper().whileTrue(IntakeCommand(-1));
    }
}


    public Command IntakeCommand(double speed){
        return runOnce(() -> intakeMotor.set(speed));
    }
    
}