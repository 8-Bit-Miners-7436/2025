package frc.robot;

import frc.robot.utils.CustomSwerveModule;
import frc.robot.utils.Elevator;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Robot extends LoggedRobot {
  public Robot() {
    // * Initialize AdvantageKit (Logger)
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    // * Start the Camera Stream
    UsbCamera camera = CameraServer.startAutomaticCapture();

    // * Configure the Camera
    camera.setResolution(640, 480);
    camera.setFPS(30);
    camera.setBrightness(20);
  }

  // * Initialize Xbox Controller, Elevator, and IMU (Gyro)
  private final XboxController gamepad = new XboxController(0);
  private final Pigeon2 pigeon = new Pigeon2(10);
  private final Elevator elevator = new Elevator();

  // * Create Kinematics Object with Module Offsets from Robot Center (in meters)
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(0.34, 0.3), // Front Left
      new Translation2d(0.34, -0.3),  // Front Right
      new Translation2d(-0.34, 0.3),  // Rear Left
      new Translation2d(-0.34, -0.3)    // Rear Right
  );

  // * Initialize Swerve Modules and packages them in an Array for Easy Access
  private final CustomSwerveModule[] modules = new CustomSwerveModule[] {
      new CustomSwerveModule(1, "Front Left"),
      new CustomSwerveModule(2, "Front Right"),
      new CustomSwerveModule(3, "Rear Left"),
      new CustomSwerveModule(4, "Rear Right")
  };

  @Override
  public void robotPeriodic() {
    // * Log Robot's Rotation/Facing Angle in AdvantageScope
    Logger.recordOutput("RobotRotation", pigeon.getRotation2d());
  }

  int elevatorLevel = 1;

  @Override
  public void teleopInit() {
    elevator.resetEncoder();
  }

  @Override
  public void teleopPeriodic() {
    // * Get Joystick Values for driving
    double vx = -gamepad.getLeftY(); // Forward/backward movement
    double vy = -gamepad.getLeftX(); // Strafing movement
    double a = -gamepad.getRightX(); // Rotational movement

    // * Convert Joystick Inputs to Chassis Speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, a);

    // * Convert Chassis Speeds to individual Swerve Module States
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // * logs Module States and Chassis Speeds in AdvantageScope
    Logger.recordOutput("ModuleStates", moduleStates);
    Logger.recordOutput("ChassisSpeeds", chassisSpeeds);

    // * Pass Swerve Module States to each Swerve Module
    for (int i = 0; i < modules.length; i++) {
      modules[i].updateState(moduleStates[i]);
    }

    elevatorLevel = elevator.moveTo(getPressedButton());
    Logger.recordOutput("Elevator Pos", elevator.getPosition());

    // * Pass Gamepad Trigger Values to Coral Launcher
    elevator.setCoralSpeed((gamepad.getLeftTriggerAxis() - gamepad.getRightTriggerAxis()) / 2);
  }
  
  private int getPressedButton() {
    if (gamepad.getAButton()) return 1;                //  A: Move to Bottom
    else if (gamepad.getBButton()) return 2;           //  B: Move to Coral Intake
    else if (gamepad.getXButton()) return 3;           //  X: Move to L2
    else if (gamepad.getYButton()) return 4;           //  Y: Move to L3 (Top)
    else if (gamepad.getLeftBumperButton()) return 5;  // LB: Manually Move Down
    else if (gamepad.getRightBumperButton()) return 6; // RB: Manually Move Up
    else return elevatorLevel;
  }

  @Override
  public void autonomousInit() {
    for (CustomSwerveModule module : modules) {
      module.resetDriveEncoder();
      module.resetSteerEncoder();
    }
  }

  @Override
  public void autonomousPeriodic() {
    if (modules[0].getEncoderDistance() > -30)
      for (int i = 0; i < modules.length; i++)
        modules[i].setDriveSpeed(-0.5);
    else
      for (int i = 0; i < modules.length; i++)
        modules[i].setDriveSpeed(0);
  }
}