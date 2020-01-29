

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous
@Disabled
public class encoderDriveTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private Servo autoGrab;
    private BNO055IMU imu;
    BNO055IMU.Parameters IMU_Parameters;

    double Left_Power;
    double Right_Power;
    float Yaw_Angle;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 45/35 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {
        BackRight = hardwareMap.dcMotor.get("FrontRight");
        BackLeft = hardwareMap.dcMotor.get("FrontLeft");
        FrontLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight = hardwareMap.dcMotor.get("BackRight");
        // reverse right motors
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        autoGrab = hardwareMap.servo.get("autoGrab");
        autoGrab.setDirection(Servo.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Create an IMU parameters object.
        IMU_Parameters = new BNO055IMU.Parameters();


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                FrontLeft.getCurrentPosition(),
                FrontRight.getCurrentPosition());
        // Set the IMU sensor mode to IMU. This mode uses
        // the IMU gyroscope and accelerometer to
        // calculate the relative orientation of hub and
        // therefore the robot.
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // Intialize the IMU using parameters object.
        imu.initialize(IMU_Parameters);
        // Report the initialization to the Driver Station.
        telemetry.addData("Status", "IMU initialized, calibration started.");
        telemetry.update();
        // Wait one second to ensure the IMU is ready.
        sleep(1000);
        // Loop until IMU has been calibrated.
        while (!IMU_Calibrated()) {
            telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
            telemetry.update();
            // Wait one second before checking calibration
            // status again.
            sleep(1000);
        }
        // Report calibration complete to Driver Station.
        telemetry.addData("Status", "Calibration Complete");
        telemetry.addData("Action needed:", "Please press the start triangle");
        telemetry.update();
        // Wait for Start to be pressed on Driver Station.
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // drive into build zone
        //encoderStrafe(DRIVE_SPEED,  12,  -12, 5.0);
        encoderDriveIMU(DRIVE_SPEED,50,50,5);
        // turn 90 degrees
        TurnWithIMU(-0.3, -90);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            /*
            BackRight
            BackLeft
            FrontLeft
            FrontRight
            */
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = BackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = FrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            BackRight.setTargetPosition(newRightBackTarget);
            BackLeft.setTargetPosition(newLeftBackTarget);
            FrontLeft.setTargetPosition(newLeftFrontTarget);
            FrontRight.setTargetPosition(newRightFrontTarget);
            // Turn On RUN_TO_POSITION

            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (BackRight.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && FrontLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            BackRight.setPower(0);
            BackLeft.setPower(0);
            FrontLeft.setPower(0);
            FrontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveIMU(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        // current angle of the robot on start of method call
        float startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;



        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            /*
            BackRight
            BackLeft
            FrontLeft
            FrontRight
            */
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = BackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = FrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            BackRight.setTargetPosition(newRightBackTarget);
            BackLeft.setTargetPosition(newLeftBackTarget);
            FrontLeft.setTargetPosition(newLeftFrontTarget);
            FrontRight.setTargetPosition(newRightFrontTarget);
            // Turn On RUN_TO_POSITION

            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (BackRight.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && FrontLeft.isBusy())) {
                // Save gyro's yaw angle
                Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw angle", Yaw_Angle);

                // amount the motor speed is reduced by to counteract change in angle
                double adjustAmount = 0.3;
                // if the robot is going in a straight line use the correction alogrithm , if turning DO NOT
                if (leftInches == rightInches) {
                    if (Yaw_Angle < startHeading - 5) {
                        // Turn left by letting right slightly overpower
                        BackRight.setPower(Math.abs(speed));
                        BackLeft.setPower(Math.abs(speed) - adjustAmount);
                        FrontLeft.setPower(Math.abs(speed) - adjustAmount);
                        FrontRight.setPower(Math.abs(speed));
                    } else if (Yaw_Angle > startHeading + 5) {
                        // Turn right by letting left slightly overpower
                        BackRight.setPower(Math.abs(speed) - adjustAmount);
                        BackLeft.setPower(Math.abs(speed));
                        FrontLeft.setPower(Math.abs(speed));
                        FrontRight.setPower(Math.abs(speed) - adjustAmount);
                    } else {
                        // Continue straight
                        BackRight.setPower(Math.abs(speed));
                        BackLeft.setPower(Math.abs(speed));
                        FrontLeft.setPower(Math.abs(speed));
                        FrontRight.setPower(Math.abs(speed));
                    }
                }
                // if the robot is turning, do not use correction algorithm
                else {
                    // simply maintain power
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            BackRight.setPower(0);
            BackLeft.setPower(0);
            FrontLeft.setPower(0);
            FrontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    // use encoders to strafe
    public void encoderStrafe(double speed,
                             double a, double b,
                             double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            /*
            BackRight
            BackLeft
            FrontLeft
            FrontRight
            */
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = BackLeft.getCurrentPosition() + (int)(b * COUNTS_PER_INCH);
            newRightBackTarget = BackRight.getCurrentPosition() + (int)(a * COUNTS_PER_INCH);
            newLeftFrontTarget = FrontLeft.getCurrentPosition() + (int)(a * COUNTS_PER_INCH);
            newRightFrontTarget = FrontRight.getCurrentPosition() + (int)(b * COUNTS_PER_INCH);

            BackRight.setTargetPosition(newRightBackTarget);
            BackLeft.setTargetPosition(newLeftBackTarget);
            FrontLeft.setTargetPosition(newLeftFrontTarget);
            FrontRight.setTargetPosition(newRightFrontTarget);
            // Turn On RUN_TO_POSITION

            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            BackRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (BackRight.isBusy() && BackLeft.isBusy() && FrontRight.isBusy() && FrontLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            BackRight.setPower(0);
            BackLeft.setPower(0);
            FrontLeft.setPower(0);
            FrontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    private void STOPMOTORS() {
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }

    // use mecanum wheels to strafe left at desired speed
    private void strafeLeft(double power) {
        FrontLeft.setPower(-power);
        BackLeft.setPower(power);
        FrontRight.setPower(power);
        BackRight.setPower(-power);
    }
    // use mecanum wheels to strafe Right at a desired speed
    private void strafeRight(double power) {
        FrontLeft.setPower(power);
        BackLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackRight.setPower(power);

    }
    public void clawDown() {
        autoGrab.setPosition(1);

    }

    public void clawUp() {
        autoGrab.setPosition(0.5);
    }
    /**
     * Function that becomes true when gyro is calibrated and
     * reports calibration status to Driver Station in the meantime.
     */
    private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", imu.getSystemStatus().toString());
        return imu.isGyroCalibrated();
    }


    // power argument is motor power
    // drive time argument is the amount of time the motors run for
    private void DriveWithImu(double power, int DriveTime) {
        // current unix time

        long MethodStartTime = System.currentTimeMillis();
        // Initialize motor power variables
        Left_Power = power;
        Right_Power = power;
        // Set motor powers to the variable values.
        FrontLeft.setPower(Left_Power);
        BackLeft.setPower(Left_Power);
        BackRight.setPower(Right_Power);
        FrontRight.setPower(Right_Power);
        // Move robot forward for 2 seconds or until stop
        // is pressed on Driver Station.
        // keep driving and adjusting while the current unix time is less than the time of method start + DriveTime
        while (!(DriveTime+MethodStartTime < System.currentTimeMillis()  || isStopRequested())) {
            // Save gyro's yaw angle
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw angle", Yaw_Angle);
            // If the robot is moving straight ahead the
            // yaw value will be close to zero. If it's not, we
            // need to adjust the motor powers to adjust heading.
            // If robot yaws right or left by 5 or more,
            // adjust motor power variables to compensation.
            if (Yaw_Angle < -5) {
                // Turn left
                Left_Power = 0.25;
                Right_Power = 0.35;
            } else if (Yaw_Angle > 5) {
                // Turn right.
                Left_Power = 0.35;
                Right_Power = 0.25;
            } else {
                // Continue straight
                Left_Power = 0.3;
                Right_Power = 0.3;
            }
            // Report the new power levels to the Driver Station.
            telemetry.addData("Left Motor Power", Left_Power);
            telemetry.addData("Right Motor Power", Right_Power);
            // Update the motors to the new power levels.
            FrontLeft.setPower(Left_Power);
            BackLeft.setPower(Left_Power);
            BackRight.setPower(Right_Power);
            FrontRight.setPower(Right_Power);
            telemetry.update();
            // Wait 1/5 second before checking again.
            sleep(200);
        }

    }
    // takes in power variable and turns until it reaches the TurnAngle variable
    // turns right if power is positive, left if negative
    public void TurnWithIMU(double power, int TurnAngle) {
        // angle of method call
        float startAngle =  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        // Now let's execute a right turn using power
        // levels that will cause a turn in place.
        BackRight.setPower(power);
        FrontRight.setPower(power);
        FrontLeft.setPower(-power);
        BackLeft.setPower(-power);
        // Continue until robot yaws right by 90 degrees
        // or stop is pressed on Driver Station.
        while (!(Yaw_Angle <= TurnAngle || isStopRequested())) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        BackRight.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
    }


}
