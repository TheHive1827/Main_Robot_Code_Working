package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.ProfiledPIDController;
import java.io.File;
import java.util.Set;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;

import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.*;
import com.ctre.phoenix6.hardware.CANcoder;
// import swervelib.encoders.CANCoderSwerve;


public class ArmSubsystem extends SubsystemBase{ // Initializing physical swerve modules
    private final static SparkMax m_ArmMotor = new SparkMax(ArmConstants.armID, MotorType.kBrushless);
  private double m_EncoderValue = m_ArmMotor.getEncoder().getPosition();
  private RelativeEncoder m_ArmEncoder = m_ArmMotor.getEncoder();
  private PIDController m_controller;
  static CommandXboxController ElevatorController = new CommandXboxController(1);
   XboxController exampleXbox = new XboxController(1); // 0 is the USB Port to be used as indicated on the Driver Station
  
  double goal;
  int SetPoint;
  double Speed;
  double Direction;
  ProfiledPIDController testname = new ProfiledPIDController(0.0,
          0.0,
          0.0, ArmConstants.MOVEMENT_CONSTRAINTS);
  //  testname.setReference(setPoint, SparkBase.ControlType.kMAXMotionPositionControl);
  public void ArmPeriodic(){
    double m_EncoderValue = m_ArmMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("arm encoder",m_EncoderValue);
    SmartDashboard.putNumber("POV Button Angle",exampleXbox.getPOV());
    SmartDashboard.putNumber("Arm Goal",goal);
    SmartDashboard.putNumber("Arm Speed",m_ArmMotor.get());
    SmartDashboard.putNumber("ArmDirection",Direction);
    double LessRange = goal - 0.01;
    double MoreRange = goal + 0.01;
    Boolean InRange;
     if (LessRange < m_EncoderValue){
      if (m_EncoderValue < MoreRange){
        InRange = true;
      } else {
        InRange = false;
      }
     } else {
      InRange = false;
     }
    // 0 = top, right = 90, bottom = 180, yada yada you should know how angles work
    if (m_EncoderValue < goal && InRange == false){
      Direction = 0.3;
      SmartDashboard.putString("Run or Nah","SHOULD BE RUNNING");
    } else if (m_EncoderValue > goal && InRange == false ){
      Direction = -0.3;
      SmartDashboard.putString("Run or Nah","SHOULD BE RUNNING");
    } else {
      Direction = 0.0;
      SmartDashboard.putString("Run or Nah","SHOULD NOT BE RUNNING");
    }
    RunMotor(Direction);
    
  }

  public void resetEncoders(){
    m_ArmEncoder.setPosition(0);
    goal = 0;
  }

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */

public void configureBindings(){
        if (exampleXbox.getPOV() == 90){
          ElevatorController.pov(90).whileTrue(SetPosition(0.4));
          goal = 0.4;
        } else {
          ;
        }

        if (exampleXbox.getPOV() == 180){
          ElevatorController.pov(-1).whileTrue(SetPosition(0.8));
          goal = 0.8;
        } else {
          ;
        }

        if (exampleXbox.getPOV() == 0){
          ElevatorController.pov(-1).whileTrue(SetPosition(0));
          goal = 0;
        } else {
          ;
        }
}

  public Command SetPosition(double setgoal) {
      return runOnce(() -> m_ArmMotor.set(testname.calculate(m_ArmEncoder.getPosition(), setgoal)));
    }

    public void RunMotor(double speed){
      m_ArmMotor.set(speed);
    }
}