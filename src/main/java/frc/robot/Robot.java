package frc.robot;
import frc.robot.utils.*;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Robot extends LoggedRobot {
  // Initialize Xbox Controller, Elevator, and IMU (Gyro)
  private final XboxController gamepad = new XboxController(0);
  private final Pigeon2 pigeon = new Pigeon2(10);
  private final Elevator elevator = new Elevator();
  private Pose3d llTranslationData;
  private double[] llRotationData;

  public Robot() {
    // Initialize AdvantageKit (Logger)
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    // Start the camera stream
    UsbCamera camera = CameraServer.startAutomaticCapture();

    // Configure The Camera
    camera.setFPS(30);
    camera.setBrightness(20);
  }

  // Create Kinematics Object w/ Module Offsets from Robot Center (in meters)
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(0.34, 0.3),   // Front Left
    new Translation2d(0.34, -0.3),  // Front Right
    new Translation2d(-0.34, 0.3),  // Rear Left
    new Translation2d(-0.34, -0.3)  // Rear Right
    );

  // Package Initialized Swerve Modules in an Array for Easy Access
  private final CustomSwerveModule[] modules = new CustomSwerveModule[] {
    new CustomSwerveModule(1, "Front Left"),
    new CustomSwerveModule(2, "Front Right"),
    new CustomSwerveModule(3, "Rear Left"),
    new CustomSwerveModule(4, "Rear Right")
  };


  @Override
  public void robotPeriodic() {
    llTranslationData = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    llRotationData = LimelightHelpers.getCameraPose_TargetSpace("limelight");
    Logger.recordOutput("LimelightData", llTranslationData);
    Logger.recordOutput("LimelightRotationData", llRotationData);
  }

  @Override
  public void teleopPeriodic() {
    // Get Joystick Values for Driving
    double vx = -gamepad.getLeftY(); // Forward/backward movement
    double vy = -gamepad.getLeftX(); // Strafing movement
    double w = -gamepad.getRightX(); // Rotational movement
    if (LimelightHelpers.getTV("limelight") && gamepad.getStartButton()) {
      w = 0.03 * llRotationData[4];
      if (Math.abs(w) < 0.1) w = 0;
      vy = (Math.abs(llRotationData[4]) > 1) ? 0 : -llTranslationData.getX();
      vx = (Math.abs(llTranslationData.getX()) > 1) ? 0 : llTranslationData.getY();
    }
    // Convert Joystick Inputs to Swerve Module Instructions (States)
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, w);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Send Swerve Diagnostics to AdvantageScope
    Logger.recordOutput("RobotRotation", pigeon.getRotation2d());
    Logger.recordOutput("ChassisSpeeds", chassisSpeeds);
    Logger.recordOutput("ModuleStates", moduleStates);
    // Pass Instructions to each Swerve Module
    int i = 0;
    for (SwerveModuleState state : moduleStates)
      modules[i++].updateState(state);

    // Move Elevator to Pressed Button's Corresponding Reef Level
    String elevatorLevel = getPressedButton();
    if (elevatorLevel != null)
      elevator.moveToLevel(elevatorLevel);
    elevator.updateElevator();

    // Set Coral Launcher Speed based on Trigger Input
    elevator.setCoralSpeed(0.5 * (gamepad.getLeftTriggerAxis() - gamepad.getRightTriggerAxis()));
  }

  // Gets Pressed Gamepad Button for Elevator Control
  private String getPressedButton() {
    if (gamepad.getAButtonReleased()) return "BASE";          //  A: Move to Bottom
    if (gamepad.getBButtonReleased()) return "INTAKE";        //  B: Move to Coral Intake
    if (gamepad.getXButtonReleased()) return "L2";            //  X: Move to L2
    if (gamepad.getYButtonReleased()) return "L3";            //  Y: Move to L3 (Top)
    if (gamepad.getLeftBumperButton()) return "DOWN"; // LB: Manually Move Down
    if (gamepad.getRightBumperButton()) return "UP";  // RB: Manually Move Up
    if (gamepad.getLeftBumperButtonReleased()) return "STOP";
    if (gamepad.getRightBumperButtonReleased()) return "STOP";
    return null;
  }

  @Override
  public void autonomousInit() {
    // Zero Out all Module Encoders to Fix Alignment Issues
    for (CustomSwerveModule module : modules)
      module.resetEncoders();
  }

  @Override
  public void testInit() {
    // Zero Out all Module Encoders to Fix Alignment Issues
    for (CustomSwerveModule module : modules)
      module.resetEncoders();
  }

  @Override
  public void autonomousPeriodic() {
    // Move Robot Directly Backward to Leave Starting Line
    if (modules[0].getEncoderDistance() > -30)
      for (CustomSwerveModule module : modules)
        module.setDriveSpeed(-0.5);
    else
      for (CustomSwerveModule module : modules)
        module.setDriveSpeed(0);
  }
}