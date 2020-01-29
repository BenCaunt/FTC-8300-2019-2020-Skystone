

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Platform and Park RED", group="Pushbot")

public class platformParkRed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor armMotor;
    private Servo autoGrab;
    private Servo clawArm;
    private Servo claw;

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
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clawArm = hardwareMap.servo.get("clawArm");
        clawArm.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");


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
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // drive into build zone
        encoderDrive(DRIVE_SPEED,  12,  12, 5.0);
        // time to strafe to build platform
        long timeToPlatform = 2350;
        // strafe to build platform
        strafeRight(0.65);
        sleep(timeToPlatform);
        STOPMOTORS();
        sleep(100);
        // lower claw
        clawDown();
        sleep(500);
        // strafe to wall
        strafeLeft(0.4);
        sleep(timeToPlatform * 3);
        STOPMOTORS();
        // release platform and raise claw
        clawUp();
        encoderDrive(DRIVE_SPEED, -28, -28, 5);
        // strafe to infront of platform
        strafeRight(0.4);
        sleep(1600);
        STOPMOTORS();
        // push platofmr back into wall
        encoderDrive(DRIVE_SPEED,  24,  24, 5.0);
        // line park
        encoderDrive(DRIVE_SPEED, -20, -20, 5);
        // strafe right into sky bridge
        strafeLeft(0.4);
        sleep(1200);
        STOPMOTORS();

        sleep(1000);     // pause for servos to move

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

}
