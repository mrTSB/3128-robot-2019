//AUTHORSES:
    //MASON HOLST
    //jude
    //caylin
    //daniel
    //tyler
    //teo
    //tanvir
    
    //not adham :/

    package org.team3128.button.main;

    import com.ctre.phoenix.motorcontrol.ControlMode;
    import com.ctre.phoenix.motorcontrol.FeedbackDevice;
    import com.ctre.phoenix.motorcontrol.can.TalonSRX;
    import com.ctre.phoenix.motorcontrol.can.VictorSPX;

    import org.team3128.athos.autonomous.*;
    import org.team3128.athos.util.PrebotDeepSpaceConstants;
    import org.team3128.button.autonomous.CmdAutoTest;
    import org.team3128.common.NarwhalRobot;
    import org.team3128.common.drive.DriveCommandRunning;
    import org.team3128.common.drive.SRXTankDrive;
    import org.team3128.common.drive.SRXTankDrive.Wheelbase;
    import org.team3128.common.drive.calibrationutility.DriveCalibrationUtility;
    import org.team3128.common.hardware.limelight.Limelight;
    import org.team3128.common.hardware.navigation.Gyro;
    import org.team3128.common.hardware.navigation.NavX;
    import org.team3128.common.util.Constants;
    import org.team3128.common.util.units.Angle;
    import org.team3128.common.util.units.Length;
    import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
    import org.team3128.gromit.util.DeepSpaceConstants;
    import org.team3128.common.util.Log;
    import org.team3128.common.util.RobotMath;
    import org.team3128.common.util.datatypes.PIDConstants;
    import org.team3128.common.narwhaldashboard.NarwhalDashboard;
    import org.team3128.common.listener.ListenerManager;
    import org.team3128.common.listener.controllers.ControllerExtreme3D;
    import org.team3128.common.listener.controltypes.Button;
    
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj.Joystick;
    import edu.wpi.first.wpilibj.RobotBase;
    import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    
    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableInstance;
    
    import java.io.BufferedWriter;
    import java.io.FileWriter;
    import java.io.File;
    import java.io.FileInputStream;
    import java.io.FileOutputStream;
    import java.io.IOException;
    import java.io.InputStream;
    import java.io.OutputStream;

    public class MainButton extends NarwhalRobot {
    
    public TalonSRX MotorOne;
    public TalonSRX MotorTwo;
    public VictorSPX OneFollower;
    public VictorSPX TwoFollower;
    public ListenerManager lm;
    public Joystick joystick;
    public SRXTankDrive tankDrive;
    public Gyro gyro;
    PIDConstants visionPID, blindPID;
    private DriveCommandRunning driveCmdRunning;
    public DriveCalibrationUtility dcu;
    public Command myCommand;

    //private DriveCommandRunning driveCmdRunning;

        @Override
        protected void constructHardware()
        {
            MotorOne = new TalonSRX(13);
            MotorTwo = new TalonSRX(15);
            OneFollower = new VictorSPX(5);
            TwoFollower = new VictorSPX(6);

            MotorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
            OneFollower.set(ControlMode.Follower, MotorOne.getDeviceID());
    
            MotorTwo.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
            TwoFollower.set(ControlMode.Follower, MotorTwo.getDeviceID());

            MotorTwo.setInverted(true);
            TwoFollower.setInverted(true);

            joystick = new Joystick(1);
            lm = new ListenerManager(joystick);
            addListenerManager(lm);
            SRXTankDrive.initialize(MotorOne, MotorTwo, 13.21*Length.in, 32.3*Length.in, 3700);
            tankDrive = SRXTankDrive.getInstance();

            gyro = new NavX();

                    // Vision
        visionPID = new PIDConstants(0, 0.02, 0.0, 0.00001);
        blindPID = new PIDConstants(0.1, 0, 0, 0);
        driveCmdRunning = new DriveCommandRunning();

        DriveCalibrationUtility.initialize(gyro, visionPID);
        dcu = DriveCalibrationUtility.getInstance();

        dcu.initNarwhalDashboard();
        }
        @Override
        protected void constructAutoPrograms() {
            NarwhalDashboard.addAuto("Square", new CmdAutoTest());
        }
    
        @Override
        protected void setupListeners() {
            lm.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		lm.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        lm.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
		

        lm.addMultiListener(() -> {
            //if (!driveCmdRunning.isRunning) {
                tankDrive.arcadeDrive(
                    -0.7 * RobotMath.thresh(lm.getAxis("MoveTurn"), 0.1),
                    -1.0 * RobotMath.thresh(lm.getAxis("MoveForwards"), 0.1),
                    -1.0 * lm.getAxis("Throttle"),
                     true
                );		
            //}
			
        }, "MoveTurn", "MoveForwards", "Throttle");
            lm.nameControl(new Button(11), "Move Forward");
            lm.addButtonUpListener("Move Forward", () -> {
                //MotorOne.set(ControlMode.PercentOutput, 0);
                tankDrive.tankDrive(0, 0);
            });
            lm.addButtonDownListener("Move Forward", () -> {
                //MotorOne.set(ControlMode.PercentOutput, 100);
                tankDrive.tankDrive(1, 1);
            });
            lm.nameControl(new Button(12), "Move Backwards");
            lm.addButtonUpListener("Move Backwards", () -> {
                //MotorOne.set(ControlMode.PercentOutput, 0);
                tankDrive.tankDrive(0,0);
            });
            lm.addButtonDownListener("Move Backwards", () -> {
                //MotorOne.set(ControlMode.PercentOutput, -100);
                tankDrive.tankDrive(-1,-1);
            });
            lm.nameControl(new Button(7), "CmdAutoTest");
            lm.addButtonDownListener("CmdAutoTest", () -> {
                myCommand = new CmdAutoTest();
                myCommand.start();
            });
        }
    
        @Override
        protected void teleopPeriodic() {
        }
    
        @Override
        protected void updateDashboard() {
            dcu.tickNarwhalDashboard();
        }
    
    
        public static void main(String... args) {
            RobotBase.startRobot(MainButton::new);
        }
    }