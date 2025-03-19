// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.IntakeConstants;

// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.sim.SparkAbsoluteEncoderSim;
// import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkMax;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkMaxAlternateEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.ctre.phoenix.motorcontrol.IFollower;
// import com.revrobotics.*;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.spark.SparkLowLevel.MotorType;


// public class IntakeSubsystem extends SubsystemBase {
//   static CommandXboxController ElevatorController = new CommandXboxController(1);
//   XboxController exampleXbox = new XboxController(1); // 0 is the USB Port to be used as indicated on the Driver Station
//   private final static SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeID, MotorType.kBrushless);

//   public void configureBindings() {
    
//       } 
//     public command IntakeCommand(){
//         return RunOnce(() -> IntakeMotor(1));
//     }
// }