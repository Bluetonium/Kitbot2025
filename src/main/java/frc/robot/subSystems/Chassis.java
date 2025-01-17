package frc.robot.subSystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
    //tank drive
    private DifferentialDrive drive;
    private SparkMax frontRightMotor;
    private SparkMax frontLeftMotor;
    private SparkMax backRightMotor;
    private SparkMax backLeftMotor;
    private SparkMaxConfig configsFrontLeft;
    private SparkMaxConfig configsFrontRight;
    private SparkMaxConfig configsBackLeft;
    private SparkMaxConfig configsBackRight;
    //constructor
    public Chassis() {

        frontRightMotor = new SparkMax(Constants.FRONT_RIGHT_MOTOR, MotorType.kBrushed);
        frontLeftMotor = new SparkMax(Constants.FRONT_LEFT_MOTOR, MotorType.kBrushed);
        backRightMotor = new SparkMax(Constants.BACK_RIGHT_MOTOR, MotorType.kBrushed);
        backLeftMotor = new SparkMax(Constants.BACK_LEFT_MOTOR, MotorType.kBrushed);


        configsFrontLeft = new SparkMaxConfig();
        configsFrontLeft.smartCurrentLimit(Constants.DRIVE_MOTOR_CURRENT_LIMIT);
        configsFrontLeft.idleMode(Constants.DRIVE_MOTOR_IDLE_MODE);

        configsFrontRight = new SparkMaxConfig();
        configsFrontRight.smartCurrentLimit(Constants.DRIVE_MOTOR_CURRENT_LIMIT);
        configsFrontRight.idleMode(Constants.DRIVE_MOTOR_IDLE_MODE);

        configsBackLeft = new SparkMaxConfig();
        configsBackLeft.smartCurrentLimit(Constants.DRIVE_MOTOR_CURRENT_LIMIT);
        configsBackLeft.idleMode(Constants.DRIVE_MOTOR_IDLE_MODE);
        configsBackLeft.follow(frontLeftMotor);

        configsBackRight = new SparkMaxConfig();
        configsBackRight.smartCurrentLimit(Constants.DRIVE_MOTOR_CURRENT_LIMIT);
        configsBackRight.idleMode(Constants.DRIVE_MOTOR_IDLE_MODE);
        configsBackLeft.follow(frontRightMotor);

        frontRightMotor.configure(configsFrontLeft,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        frontLeftMotor.configure(configsFrontRight,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        backRightMotor.configure(configsBackRight,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        backLeftMotor.configure(configsBackLeft,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    
    }
    public void drive(double speed,double rotationSpeed){
        drive.arcadeDrive(speed, rotationSpeed);
    }
}
