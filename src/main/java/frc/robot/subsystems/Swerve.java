package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class Swerve extends SubsystemBase {
    // Initialize IMU
    private final AHRS mImu = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveModule mLeftFrontModule, mRightFrontModule, mLeftRearModule, mRightRearModule;
    private SwerveDriveOdometry mOdometry;

    public Swerve() {
        // Instantiate swerve modules - each representing unique module on the robot
        mLeftFrontModule = new SwerveModule(
            SwerveConstants.kLeftFrontThrottleID, 
            SwerveConstants.kLeftFrontRotorID, 
            SwerveConstants.kLeftFrontCANCoderID, 
            SwerveConstants.kLeftFrontRotorOffset
        );

        mRightFrontModule = new SwerveModule(
            SwerveConstants.kRightFrontThrottleID, 
            SwerveConstants.kRightFrontRotorID, 
            SwerveConstants.kRightFrontCANCoderID, 
            SwerveConstants.kRightFrontRotorOffset
        );

        mLeftRearModule = new SwerveModule(
            SwerveConstants.kLeftRearThrottleID, 
            SwerveConstants.kLeftRearRotorID, 
            SwerveConstants.kLeftRearCANCoderID, 
            SwerveConstants.kLeftRearRotorOffset
        );

        mRightRearModule = new SwerveModule(
            SwerveConstants.kRightRearThrottleID, 
            SwerveConstants.kRightRearRotorID, 
            SwerveConstants.kRightRearCANCoderID, 
            SwerveConstants.kRightRearRotorOffset
        );

        // Instantiate odometry - used for tracking position
        mOdometry = new SwerveDriveOdometry(SwerveConstants.kSwerveKinematics, mImu.getRotation2d(), getModulePositions());
    }

    @Override
    public void periodic() {
        // Updates odometry with current module state
        mOdometry.update(
            mImu.getRotation2d(), 
            getModulePositions()
        );
    }

    /**
     * Drives the swerve - Input range: [-1, 1]
     * 
     * @param xSpeed percent power in the X direction
     * @param ySpeed percent power in the Y direction 
     * @param zSpeed percent power for rotation (旋轉的功率百分比)
     * @param fieldOriented configure robot movement style (field or robot oriented)
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
        SwerveModuleState[] states = null;
        if(fieldOriented) {
            states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                // IMU used for field oriented control
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, mImu.getRotation2d())
            );
        } else {
            states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
            );
        }

        // SmartDashboard.putString("Field chassis speeds: ", ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, mImu.getRotation2d()).toString());
        setModuleStates(states);
    }

    /**
     * Get current swerve module states
     * 
     * @return swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            mLeftFrontModule.getState(), 
            mRightFrontModule.getState(), 
            mLeftRearModule.getState(), 
            mRightRearModule.getState()
        };
    }

    /**
     * Get current swerve module positions
     * 
     * @return swerve module positions 
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            mLeftFrontModule.getPosition(), 
            mRightFrontModule.getPosition(), 
            mLeftRearModule.getPosition(), 
            mRightRearModule.getPosition()
        };
    }

    /**
     * Sets swerve module states+
     * 
     * @param desiredStates array of desired states, order: [leftFront, leftRear, rightFront, rightRear]
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        mLeftFrontModule.setState(desiredStates[0]);
        mRightFrontModule.setState(desiredStates[1]);
        mLeftRearModule.setState(desiredStates[2]);
        mRightRearModule.setState(desiredStates[3]);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(SwerveConstants.kSwerveKinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Get predicted pose
     * 
     * @return pose
     */
    public Pose2d getPose() {
        // SmartDashboard.putString("Get robot pose:  ", mOdometry.getPoseMeters().toString());
        return mOdometry.getPoseMeters();
    }

    /**
     * Set robot pose
     * 
     * @param pose robot pose
     */
    public void setPose(Pose2d pose) {
        mOdometry.resetPosition(mImu.getRotation2d(), getModulePositions(), pose);
    }
}
