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
  private final static SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ElevatorLeader, MotorType.kBrushless);
  private final static RelativeEncoder elevatorMotorEncoder = elevatorMotor.getEncoder();
  public static SparkClosedLoopController m_elevatorPID = elevatorMotor.getClosedLoopController();
  double ElevatorEncoderValue = elevatorMotorEncoder.getPosition();
  boolean ButtonA = exampleXbox.getAButtonPressed();
  boolean ButtonY = exampleXbox.getYButtonPressed();
  boolean ButtonX = exampleXbox.getXButtonPressed();
  boolean ButtonB = exampleXbox.getBButtonPressed();

  public void ElevatorInit(){
    final SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
    .p(0.000001)
    .i(0.0)
    .d(0.0)
    .velocityFF(1/473)
    .outputRange(ElevatorConstants.ElevatorMin, ElevatorConstants.ElevatorMax);
    elevatorMotor.configure(config, null, null);
    elevatorMotorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    ButtonA = exampleXbox.getAButtonPressed();
    ButtonY = exampleXbox.getYButtonPressed();
    SmartDashboard.putNumber("Elevator encoder", elevatorMotorEncoder.getPosition());
    SmartDashboard.putBoolean("A", ButtonA);
    SmartDashboard.putBoolean("Y", ButtonY);}

  public Command ElevatorCommand(double speed) {
    return runOnce(() -> elevatorMotor.set(speed));
    }

  public Command ElevatorDown(double speed) {
    return runOnce(() -> elevatorMotor.set(-speed));
  }

  public Command ElevatorGoToPoint(double setPoint){
    return runOnce(() -> m_elevatorPID.setReference(setPoint, ControlType.kVelocity));
  }


    public void configureBindings() {
        ButtonA = exampleXbox.getAButtonPressed();
        ButtonY = exampleXbox.getYButtonPressed();
        ButtonX = exampleXbox.getXButtonPressed();
          if (ButtonA == false && ButtonY == false && ButtonX == false){
            ElevatorController.y().whileFalse(ElevatorCommand(0));
            ElevatorController.a().whileFalse(ElevatorCommand(0));
            ElevatorController.x().whileFalse(ElevatorGoToPoint(0));
            // when both of them are NOT being pressed, when either of them are NOT ebing pressed it stops the motors.
          }
           else {
            ElevatorController.a().whileTrue(ElevatorCommand(-0.1));
            ElevatorController.y().whileTrue(ElevatorCommand(0.1));
            ElevatorController.x().whileFalse(ElevatorGoToPoint(ElevatorConstants.ElevatorMax));
            // 
          } 
        
      }
}