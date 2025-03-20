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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
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

public class ArmSubsystem extends SubsystemBase{
// i love dry paint2b .

  private final static SparkMax m_ArmMotor = new SparkMax(ArmConstants.armID, MotorType.kBrushless);
  private double m_EncoderValue = m_ArmMotor.getEncoder().getPosition();
  private RelativeEncoder m_ArmEncoder = m_ArmMotor.getEncoder();
  // private PIDController m_controller;
  SparkClosedLoopController m_closedloop = m_ArmMotor.getClosedLoopController();
  static CommandXboxController ElevatorController = new CommandXboxController(1);
   XboxController exampleXbox = new XboxController(1); // 0 is the USB Port to be used as indicated on the Driver Station
  SparkMaxConfig config = new SparkMaxConfig();
  
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;


// Set PID gains
// PIDController PIDController = new PIDController(
//             SwerveConstants.kRotor_kP,
//             SwerveConstants.kRotor_kI,
//             SwerveConstants.kRotor_kD
//         );
  double goal;
  int SetPoint;
  double Speed;
  double Direction;

  public void config(){
    m_closedloop = m_ArmMotor.getClosedLoopController();
    encoder = m_ArmMotor.getEncoder();

    motorConfig = new SparkMaxConfig();

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(2.65)
        // speed
        .i(0.0)
        // integral 
        .d(0.0)
        // kinda like friction
        .outputRange(-1.0, 0.3);

    m_ArmMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // ProfiledPIDController testname = new ProfiledPIDController(0.0,
  //         0.0,
  //         0.0, ArmConstants.MOVEMENT_CONSTRAINTS);

  public void ArmPeriodic(){
    double m_EncoderValue = m_ArmMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("arm encoder", m_EncoderValue);
    SmartDashboard.putNumber("POV Button Angle", exampleXbox.getPOV());
    SmartDashboard.putNumber("Arm Goal", goal);
    SmartDashboard.putNumber("Arm Output",  m_ArmMotor.getAppliedOutput());
    SmartDashboard.putNumber("ArmDirection", Direction);
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
          goal = ArmConstants.armMid;
        }

        if (exampleXbox.getPOV() == 180){
          goal = ArmConstants.armDown;

        }

        if (exampleXbox.getPOV() == 0){
          goal = ArmConstants.armUp;
        }
}

  public Command SetPosition(double setgoal) {
      return runOnce(() -> m_closedloop.setReference(setgoal, ControlType.kPosition));
    }

    public void RunMotor(double speed){
      m_ArmMotor.set(speed);
    }
}