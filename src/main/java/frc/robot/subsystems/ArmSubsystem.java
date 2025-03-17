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
  int Position;
  int SetPoint;
  double Speed;
  ProfiledPIDController testname = new ProfiledPIDController(0.0,
          0.0,
          0.0, ArmConstants.MOVEMENT_CONSTRAINTS);
//    testname.setReference(setPoint, SparkBase.ControlType.kMAXMotionPositionControl);
  public void Periodic(){
    double m_EncoderValue = m_ArmMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("arm encoder",m_EncoderValue);
    
  }

  public void resetEncoders(){
    m_ArmEncoder.setPosition(0);
  }

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */



  public Command armCommand(Double Voltage) {
    return runOnce(() -> m_ArmMotor.set(testname.calculate(m_ArmEncoder.getPosition(), 223232)))
    .andThen(()-> m_ArmMotor.set(1.0));
    }

  public void start(){
    Position = 1;
  }
}