/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.studica.frc.TitanQuad;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.MultipleCommands;
import frc.robot.GamepadConstants;
import frc.robot.commands.driveCommands.SimpleDrive;
import frc.robot.commands.driveCommands.SimpleServo;
import frc.robot.commands.driveCommands.StopMotors;
import com.studica.frc.Servo;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private String CurrentMode = "none";
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(new TitanQuad(42,0), new TitanQuad(42,2));
  private final Joystick m_stick = new Joystick(GamepadConstants.DRIVE_USB_PORT);

  private Servo m_servo = new Servo(1);
  Thread m_visionThread;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraInit();//remove this if you dont need cameras
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    CurrentMode = "none";
    //Check to see if autoChooser has been created
    if(null == RobotContainer.autoChooser)
    {
      RobotContainer.autoChooser = new SendableChooser<>();
    }

    //Add the default auto to the auto chooser
    RobotContainer.autoChooser.setDefaultOption("Multiple Commands", "Multiple Commands");
    RobotContainer.autoMode.put("Multiple Commands", new MultipleCommands());

    //Add other autos to the chooser
    addAutoToSelector(RobotContainer.autoChooser, "Multiple Commands", new MultipleCommands());

    //Update Smartdashboard
    SmartDashboard.putData(RobotContainer.autoChooser);
  }

  private void addAutoToSelector(SendableChooser<String> chooser, String auto, AutoCommand cmd)
  {
    chooser.addOption(auto, auto);
    RobotContainer.autoMode.put(auto, cmd);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    CurrentMode = "auto";
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double CurrentAngle = m_servo.getAngle();
    while(m_stick.getPOV(0)==90){
      CurrentAngle+=1;
      m_servo.setAngle(CurrentAngle);
    }
    while(m_stick.getPOV(0)==90){
      CurrentAngle+=1;
      m_servo.setAngle(CurrentAngle);
    }
    while(m_stick.getPOV(0)==270){
      CurrentAngle-=1;
      m_servo.setAngle(CurrentAngle);
    }
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());


  }

  @Override
  public void teleopInit() {
    CurrentMode = "teleop";
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }   
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double CurrentAngle = m_servo.getAngle();

    while(m_stick.getPOV(0)==90){
      CurrentAngle+=1;
      m_servo.setAngle(CurrentAngle);
    }
    while(m_stick.getPOV(0)==270){
      CurrentAngle-=1;
      m_servo.setAngle(CurrentAngle);
    }
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
  void CameraInit(){
    m_visionThread =
    new Thread(
        () -> {
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
          // Set the resolution
          camera.setResolution(640, 480);
          camera.setWhiteBalanceAuto();

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getInstance().getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();

          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }
            /*Shuffleboard.getTab("DRIVE").add("LOWRED", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min",0,"max",255))
            .getEntry();*/

            Scalar LowBlue = new Scalar(110, 50, 50);
            Scalar HighBlue = new Scalar(130, 255, 255);
            Mat Hsv = new Mat();
            Imgproc.cvtColor(mat,Hsv, Imgproc.COLOR_BGR2HSV);
            Mat Mask = new Mat();
            Mat hierarchy = new Mat();
            Core.inRange(Hsv, LowBlue, HighBlue, mat);
            // Put a rectangle on the image
            if(CurrentMode=="auto"){
            
              Imgproc.rectangle(
                mat, new Point(100, 100), new Point(300, 400), new Scalar(255, 55, 0), 5);
              List<MatOfPoint> contours = new ArrayList<>();
              Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
              int Area = 0;
              //for(int i=0;i<=contours.size();i++){
              //  Area += Imgproc.contourArea(contours.get(i));
              //}
              SmartDashboard.putNumber("DETECT", contours.size());
            }
              Imgproc.rectangle(
                mat, new Point(50, 50), new Point(400, 400), new Scalar(255, 255, 255), 5);
            

            // Give the output stream a new image to display
            outputStream.putFrame(mat);
          }
        });
m_visionThread.setDaemon(true);
m_visionThread.start();
  }
}
