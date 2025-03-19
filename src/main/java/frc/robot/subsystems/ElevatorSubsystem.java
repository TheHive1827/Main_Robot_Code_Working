package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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


public class ElevatorSubsystem extends SubsystemBase {
    static CommandXboxController ElevatorController = new CommandXboxController(1);
  XboxController exampleXbox = new XboxController(1); // 0 is the USB Port to be used as indicated on the Driver Station
  public final static SparkMax elevatorMotorLeader = new SparkMax(ElevatorConstants.ElevatorLeader, MotorType.kBrushless);
  SparkClosedLoopController elevatorPID = elevatorMotorLeader.getClosedLoopController();
  public static SparkMaxConfig config = new SparkMaxConfig();
  private final static AbsoluteEncoder motorLeaderEncoder = elevatorMotorLeader.getAbsoluteEncoder();
  private final static int setPoint = 0;
  double ElevatorEncoderValue = motorLeaderEncoder.getPosition();
  boolean ButtonA = exampleXbox.getAButtonPressed();
  boolean ButtonY = exampleXbox.getYButtonPressed();
  boolean ButtonX = exampleXbox.getXButtonPressed();
  boolean ButtonB = exampleXbox.getBButtonPressed();

  public ElevatorSubsystem(){
    config.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(0.01)
    .i(0.0)
    .d(0.0)
    .outputRange(-1, 1);
    elevatorPID.setReference(setPoint, ControlType.kPosition);

    elevatorMotorLeader.configure(config, null, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ButtonA = exampleXbox.getAButtonPressed();
        ButtonY = exampleXbox.getYButtonPressed();
        SmartDashboard.putNumber("elevator encoder ig",ElevatorEncoderValue);
        SmartDashboard.putBoolean("A", ButtonA);
        SmartDashboard.putBoolean("Y", ButtonY);}

  public Command ElevatorCommand(double speed) {
    return runOnce(() -> elevatorMotorLeader.set(speed));
    }

    public void configureBindings() {
        ButtonA = exampleXbox.getAButtonPressed();
        ButtonY = exampleXbox.getYButtonPressed();
          if (ButtonA == false && ButtonY == false){
            ElevatorController.y().whileFalse(ElevatorCommand(0));
            ElevatorController.a().whileFalse(ElevatorCommand(0));
            // when both of them are NOT being pressed, when either of them are NOT ebing pressed it stops the motors.
          }
           else {
            ElevatorController.a().whileTrue(ElevatorCommand(-0.1));
            ElevatorController.y().whileTrue(ElevatorCommand(0.1));
            // 
          } 
        
      }
}