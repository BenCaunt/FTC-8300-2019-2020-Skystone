package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Disabled
public class stackMechanismDemo extends LinearOpMode {
    private Servo autoGrab;
    private DcMotor armMotor;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo clawArm;
    private Servo claw;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {
        autoGrab = hardwareMap.servo.get("autoGrab");
        autoGrab.setDirection(Servo.Direction.REVERSE);
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clawArm = hardwareMap.servo.get("clawArm");
        clawArm.setDirection(Servo.Direction.REVERSE);
        autoGrab = hardwareMap.servo.get("autoGrab");
        autoGrab.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");

        waitForStart();
        // lift arm
        liftPosition(1,8,5);
        // claw out
        armIn();
        sleep(250);
        // open claw
        openClaw();
        sleep(1000);
        armOut();
        sleep(300);
        liftPosition(1,-8,5);



    }
    public void armOut() {
        clawArm.setPosition(0.73);
    }

    public void armIn() {
        clawArm.setPosition(0.05);
    }

    public void autoClawDown() {
        autoGrab.setPosition(1);

    }
    public void autoClawUp() {
        autoGrab.setPosition(0.5);
    }

    private void closeClaw() {
        claw.setPosition(1);
    }


    private void openClaw() {
        claw.setPosition(0.5);
    }
    public void liftPosition(double speed, double inches, double timeoutS) {
        int target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = armMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            armMotor.setTargetPosition(target);
            // Turn On RUN_TO_POSITION

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            armMotor.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (armMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d ", target);
                telemetry.addData("Path2",  "Running at %7d ",
                        armMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            armMotor.setPower(0);


            // Turn off RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }



}
