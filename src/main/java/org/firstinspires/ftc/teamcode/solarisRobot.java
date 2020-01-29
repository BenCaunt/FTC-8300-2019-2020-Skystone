package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class solarisRobot {
    /* Declare OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor BackRight;
    public DcMotor BackLeft;
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public Servo autoGrab;
    public Servo claw;
    public Servo clawArm;
    public DcMotor armMotor;




    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = 0.6;
    public static final double     TURN_SPEED              = 0.5;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* class construction */

    public solarisRobot(){

    }

    /* init hardware map */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // init motors
        BackRight = hwMap.get(DcMotor.class, "FrontRight");
        BackLeft = hwMap.get(DcMotor.class, "FrontLeft");
        FrontLeft = hwMap.get(DcMotor.class, "BackLeft");
        armMotor = hwMap.get(DcMotor.class, "armMotor");

        FrontRight = hwMap.get(DcMotor.class, "BackRight");
        // init servos
        clawArm = hwMap.get(Servo.class, "clawArm");
        clawArm.setDirection(Servo.Direction.REVERSE);
        autoGrab = hwMap.get(Servo.class, "autoGrab");
        autoGrab.setDirection(Servo.Direction.REVERSE);
        claw = hwMap.get(Servo.class, "claw");


    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

}
