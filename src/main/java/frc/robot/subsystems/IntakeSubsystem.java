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
  private final static SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeID, MotorType.kBrushless);

  public void configureBindings() {
    // When either bumper changes to false, stop the motor.
    ElevatorController.leftBumper().onFalse(IntakeCommand(0));
    ElevatorController.rightBumper().onFalse(IntakeCommand(0));
    // While either bumper is true, run the motor.
    ElevatorController.leftBumper().onTrue(IntakeCommand(1));
    ElevatorController.rightBumper().onTrue(IntakeCommand(-1));
  }

  public Command IntakeCommand(double speed) {
    return runOnce(() -> intakeMotor.set(speed));
  }

}