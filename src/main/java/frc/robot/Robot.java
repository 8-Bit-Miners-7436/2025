package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.CustomSwerveModule;

public class Robot extends TimedRobot {
  // * Initialize Xbox Controller and IMU (Gyro)
  private final XboxController gamepad = new XboxController(0);
  private final Pigeon2 pigeon = new Pigeon2(10);

  // * Create Kinematics Object with Module Offsets from Robot Center (in meters)
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(0.34, 0.3), // Front Left
      new Translation2d(0.34, -0.3), // Front Right
      new Translation2d(-0.34, 0.3), // Rear Left
      new Translation2d(-0.34, -0.3) // Rear Right
  );

  // * Initialize Swerve Modules and packages them in an Array for Easy Access
  private final CustomSwerveModule[] modules = new CustomSwerveModule[] {
      new CustomSwerveModule(1, "Front Left"),
      new CustomSwerveModule(2, "Front Right"),
      new CustomSwerveModule(3, "Rear Left"),
      new CustomSwerveModule(4, "Rear Right")
  };

  // Todo: Implement a more permanent elevator solution
  private final SparkMax elevatorMotor = new SparkMax(15, MotorType.kBrushless);

  @Override
  public void robotPeriodic() {
    // * Retrieve and normalize Pigeon IMU angles (Yaw, Pitch, Roll)
    double pigeonYaw = pigeon.getYaw(true).getValueAsDouble() % 360;
    double pigeonPitch = pigeon.getPitch(true).getValueAsDouble() % 360;
    double pigeonRoll = pigeon.getRoll(true).getValueAsDouble() % 360;

    // * Display IMU values on the SmartDashboard
    SmartDashboard.putNumber("Yaw", pigeonYaw);
    SmartDashboard.putNumber("Pitch", pigeonPitch);
    SmartDashboard.putNumber("Roll", pigeonRoll);
  }

  @Override
  public void teleopPeriodic() {
    // * Get Joystick Values for driving
    double vx = gamepad.getLeftY(); // Forward/backward movement
    double vy = gamepad.getLeftX(); // Strafing movement
    double a = gamepad.getRightX(); // Rotational movement

    // * Convert Joystick Inputs to Chassis Speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, a);

    // * Convert Chassis Speeds to individual Swerve Module States
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // * Stores Swerve Module States inside each Module Object
    for (int i = 0; i < modules.length; i++)
      modules[i].state = moduleStates[i];

    // * Loop through each Swerve Module, update Motor Speeds and Target Angles
    for (CustomSwerveModule module : modules) {
      double targetAngle = module.state.angle.getRotations();
      double targetSpeed = module.state.speedMetersPerSecond;

      // * Publish Target Angle and Speed to SmartDashboard for debugging
      SmartDashboard.putNumber(module.name + " Target Angle", targetAngle);
      SmartDashboard.putNumber(module.name + " Target Speed", targetSpeed);

      // * Passes Target Speed and Angle to Swerve Module
      module.setDriveSpeed(targetSpeed);
      module.setTargetAngle(targetAngle);
    }

    // Todo: Implement a more permanent elevator solution
    if (gamepad.getAButton()) {
      elevatorMotor.set(1);
    } else if (gamepad.getBButton()) {
      elevatorMotor.set(-1);
    } else {
      elevatorMotor.stopMotor();
    }
  }

  @Override
  public void testInit() {
    // * Resets All Swerve Module Steer-Motor Encoders
    // * Enable Test Mode briefly after manually re-aligning wheels
    for (CustomSwerveModule module : modules) {
      module.resetEncoder();
    }
  }
}