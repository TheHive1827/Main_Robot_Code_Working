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
// i love dry paint

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
  double goal = 0;
  int SetPoint;
  double Speed;
  double Direction;

  public void config(){
    closedLoopController = m_ArmMotor.getClosedLoopController();
    encoder = m_ArmMotor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    // motorConfig.encoder
    //     .positionConversionFactor(1)
    //     .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(1.0)
        .i(0.1)
        .d(0)
        .outputRange(-1, 1);
        // // Set PID values for velocity control in slot 1
        // .p(0.0001, ClosedLoopSlot.kSlot1)
        // .i(0, ClosedLoopSlot.kSlot1)
        // .d(0, ClosedLoopSlot.kSlot1)
        // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    m_ArmMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    // SmartDashboard.setDefaultNumber("Target Position", 0);
    // SmartDashboard.setDefaultNumber("Target Velocity", 0);
    // SmartDashboard.setDefaultBoolean("Control Mode", false);
    // SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  // ProfiledPIDController testname = new ProfiledPIDController(0.0,
  //         0.0,
  //         0.0, ArmConstants.MOVEMENT_CONSTRAINTS);
  public void ArmPeriodic(){
    double m_EncoderValue = m_ArmMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("arm encoder",m_EncoderValue);
    SmartDashboard.putNumber("POV Button Angle",exampleXbox.getPOV());
    SmartDashboard.putNumber("Arm Goal",goal);
    SmartDashboard.putNumber("Arm Output", m_ArmMotor.getAppliedOutput());
    // SmartDashboard.putNumber("ArmDirection",Direction);
    // double LessRange = goal - 0.05;
    // double MoreRange = goal + 0.05;
    // Boolean InRange;

    // m_ArmMotor.set(ElevatorController.getLeftY());

    m_closedloop.setReference(goal, ControlType.kPosition);

    
    //  if (LessRange < m_EncoderValue){
    //   if (m_EncoderValue < MoreRange){
    //     InRange = true;
    //   } else {
    //     InRange = false;
    //   }
    //  } else {
    //   InRange = false;
    //  }
    // // 0 = top, right = 90, bottom = 180, yada yada you should know how angles work
    // if (m_EncoderValue < goal && InRange == false){
    //   Direction = 0.1;
    //   SmartDashboard.putString("Run or Nah","SHOULD BE RUNNING");
    // } else if (m_EncoderValue > goal && InRange == false ){
    //   Direction = -0.75;
    //   SmartDashboard.putString("Run or Nah","SHOULD BE RUNNING");
    // } else {
    //   Direction = 0.0;
    //   SmartDashboard.putString("Run or Nah","SHOULD NOT BE RUNNING");
    // }
    // RunMotor(Direction);

    
    
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
          // ElevatorController.pov(90).whileTrue(SetPosition(0.2));
          goal = 0.2;
        }

        if (exampleXbox.getPOV() == 180){
          // ElevatorController.pov(-1).whileTrue(SetPosition(1.2));
          goal = 1.2;
        }

        if (exampleXbox.getPOV() == 0){
          // ElevatorController.pov(-1).whileTrue(SetPosition(0));
          goal = 0;
        }
}

  public Command SetPosition(double setgoal) {
      return runOnce(() -> m_closedloop.setReference(setgoal, ControlType.kPosition));
    }

    public Command RunMotor(double speed){
      return runOnce(() -> m_ArmMotor.set(speed));
    }
}