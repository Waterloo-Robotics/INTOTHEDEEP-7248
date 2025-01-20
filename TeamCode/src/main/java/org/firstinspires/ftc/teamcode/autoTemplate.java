package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous
public class autoTemplate extends LinearOpMode {

    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx rearLeft;
    DcMotorEx rearRight;

    public DcMotorEx rightSlide  = null;
    public DcMotorEx leftSlide  = null;

    public Servo intake_claw = null;
    public Servo intake_claw_orientation = null;
    public Servo intake_claw_rotation = null;
    public Servo intake_arm_rotation_right = null;
    public Servo intake_arm_rotation_left = null;
    public Servo intake_slider = null;
    public Servo scoring_arm_right = null;
    public Servo scoring_arm_left = null;
    public Servo scoring_claw = null;

    static final double     FORWARD_SPEED = 0.3;
    static final double     REVERSE_SPEED = 0.3;
    static final double     TURN_SPEED    = 0.7;
    static final double     SPIN_SPEED = 0.4;
    static final double COUNTS_PER_REV = 553;
    static final double GEAR_RATIO = 1.0;
    static final double WHEEL_DIAMETER_MM = 104.0;
    static final double WHEEL_CIRCUMFERENCE_MM = Math.PI * WHEEL_DIAMETER_MM;
    static final double COUNTS_PER_INCH = COUNTS_PER_REV / (WHEEL_CIRCUMFERENCE_MM * GEAR_RATIO / 25.4);

    @Override
    public void runOpMode() {

        frontLeft  = (DcMotorEx) hardwareMap.get(DcMotor.class, "LD");
        frontRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "RD");
        rearLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "bldr");
        rearRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "brdr");
        frontLeft.setTargetPositionTolerance(15);
        frontRight.setTargetPositionTolerance(15);
        rearLeft.setTargetPositionTolerance(15);
        rearRight.setTargetPositionTolerance(15);
//
        rightSlide = (DcMotorEx) hardwareMap.get(DcMotor.class, "R_slide");
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide = (DcMotorEx) hardwareMap.get(DcMotor.class, "L_slide");

        rightSlide.setTargetPosition(0);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake_claw = hardwareMap.get(Servo.class, "intake_claw");
        intake_claw_orientation = hardwareMap.get(Servo.class, "intake_claw_orientation");
        intake_claw_rotation = hardwareMap.get(Servo.class, "intake_claw_rotation");
        intake_arm_rotation_right = hardwareMap.get(Servo.class, "intake_arm_rotation_r");
        intake_arm_rotation_left = hardwareMap.get(Servo.class, "intake_arm_rotation_l");
        intake_slider = hardwareMap.get(Servo.class, "intake_slider");
        scoring_arm_right = hardwareMap.get(Servo.class, "scoring_arm_right");
        scoring_arm_left = hardwareMap.get(Servo.class, "scoring_arm_left");
        scoring_claw = hardwareMap.get(Servo.class, "scoring_claw");

        intake_claw.setPosition(Constants.INTAKE_CLOSE);
        intake_claw_orientation.setPosition(Constants.ORIENTATION_HOME);
        intake_claw_rotation.setPosition(Constants.INTAKE_CLAW_ROTATION_HOME);
        intake_arm_rotation_left.setPosition(1 - Constants.INTAKE_ROTATION_HOME);
        intake_arm_rotation_right.setPosition(Constants.INTAKE_ROTATION_HOME);
        intake_slider.setPosition(Constants.SLIDER_HOME);
        scoring_arm_left.setPosition(Constants.SCORING_ARM_HOME);
        scoring_arm_right.setPosition(Constants.SCORING_ARM_HOME);
        scoring_claw.setPosition(Constants.SCORING_CLOSE);

        waitForStart();



    }

    public void encoderDriveAuto(double fl, double fr, double rl, double rr, double timeout) {

        ElapsedTime time = new ElapsedTime();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int frontLeftPos = (int) (fl * COUNTS_PER_INCH);
        int frontRightPos = (int) (fr * COUNTS_PER_INCH);
        int rearLeftPos = (int) (rl * COUNTS_PER_INCH);
        int rearRightPos = (int) (rr * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        rearLeft.setTargetPosition(rearLeftPos);
        rearRight.setTargetPosition(rearRightPos);

        time.reset();

        while ((frontLeft.isBusy() || frontRight.isBusy() || rearLeft.isBusy() || rearRight.isBusy()) && time.seconds() < timeout && !isStopRequested()) {
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            rearLeft.setPower(0.5);
            rearRight.setPower(0.5);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

    }

}
