/*
modified from pxt-servo/servodriver.ts
load dependency
"motorbit": "file:../pxt-motorbit"
*/
enum Offset {
    //% block=one
    ONE = 0,
    //% block=two
    TWO = 1,
    //% block=three
    THREE = 2,
    //% block=four
    FOUR = 3
}

//% color="#EE6A50" weight=10 icon="\uf0d1"
//% groups=['Gorilla Go', 'Motor', 'Servo', 'GeekServo', 'Stepper Motor', 'RUS-04', 'RGB']
namespace motorbit {

    // ── PCA9685 constants ─────────────────────────────────────────────────────────
    const PCA9685_ADDRESS = 0x40
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09
    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    const STP_CHA_L = 2047
    const STP_CHA_H = 4095
    const STP_CHB_L = 1
    const STP_CHB_H = 2047
    const STP_CHC_L = 1023
    const STP_CHC_H = 3071
    const STP_CHD_L = 3071
    const STP_CHD_H = 1023

    // ── BNO055 constants ──────────────────────────────────────────────────────────
    const BNO055_ADDR = 0x28
    const BNO055_OPR_MODE_REG = 0x3D
    const BNO055_EUL_H_LSB = 0x1A

    // ── Enums ─────────────────────────────────────────────────────────────────────
    export enum Servos {
        S1 = 0x01,
        S2 = 0x02,
        S3 = 0x03,
        S4 = 0x04,
        S5 = 0x05,
        S6 = 0x06,
        S7 = 0x07,
        S8 = 0x08
    }

    export enum Motors {
        M1 = 0x1,
        M2 = 0x2,
        M3 = 0x3,
        M4 = 0x4
    }

    export enum Steppers {
        STPM1_2 = 0x2,
        STPM3_4 = 0x1
    }

    export enum SonarVersion {
        V1 = 0x1,
        V2 = 0x2
    }

    export enum Turns {
        //% blockId="T1B4" block="1/4"
        T1B4 = 90,
        //% blockId="T1B2" block="1/2"
        T1B2 = 180,
        //% blockId="T1B0" block="1"
        T1B0 = 360,
        //% blockId="T2B0" block="2"
        T2B0 = 720,
        //% blockId="T3B0" block="3"
        T3B0 = 1080,
        //% blockId="T4B0" block="4"
        T4B0 = 1440,
        //% blockId="T5B0" block="5"
        T5B0 = 1800
    }

    export enum DistanceUnit {
        //% block="cm"
        CM = 0,
        //% block="inch"
        Inch = 1
    }

    // ── State variables ───────────────────────────────────────────────────────────
    let initialized = false
    let matBuf = pins.createBuffer(17);
    let distanceBuf = 0;

    let gg_leftMotor: Motors = Motors.M1
    let gg_rightMotor: Motors = Motors.M2
    let gg_leftWheelDia: number = 4
    let gg_rightWheelDia: number = 4
    let gg_trackWidth: number = 9.4
    let gg_ticksPerRev: number = 270
    let gg_leftTicks: number = 0
    let gg_rightTicks: number = 0
    let gg_leftMotorDir: number = -1
    let gg_rightMotorDir: number = 1
    let gg_zeroHeading: number = 0
    let gg_calBuf = pins.createBuffer(22)
    let gg_calSaved = false

    // ── Drive Straight tuning (default; ปรับผ่านบล็อก Set Drive Tuning ได้) ──
    let gg_kp: number = 1.2         // เกน P การคุมตรง
    let gg_ki: number = 0.05        // เกน I
    let gg_minSpeed: number = 45    // PWM ต่ำสุดพ้น deadzone
    let gg_rampTicks: number = 60   // ระยะ ramp เข้า/ออก (ticks)

    // ── Turn tuning (default; ปรับผ่านบล็อก Set Turn Tuning ได้) ──
    let gg_turnMidSpeed: number = 65    // speed เฟส 2 (หมุนช้าคงที่)
    let gg_turnCoarseExit: number = 40  // เกณฑ์จบเฟส 1 (°)
    let gg_turnMidExit: number = 10     // เกณฑ์จบเฟส 2 (°)
    let gg_turnFineTol: number = 0.5    // ความละเอียดเฟส 3 (°)

    let gg_leftArmServo: Servos = Servos.S2
    let gg_rightArmServo: Servos = Servos.S1
    let gg_leftArmOpenAngle: number = 0
    let gg_rightArmOpenAngle: number = 210
    let gg_leftArmCloseAngle: number = 210
    let gg_rightArmCloseAngle: number = 0
    let gg_currentArmPercent: number = 0   // 0 = เปิดสุด, 100 = ปิดสุด
    let gg_currentLeftArmPercent: number = 0   // เปอร์เซ็นต์ปัจจุบันของแขนซ้าย (แยกอิสระ)
    let gg_currentRightArmPercent: number = 0  // เปอร์เซ็นต์ปัจจุบันของแขนขวา (แยกอิสระ)

    // ── Private helpers ───────────────────────────────────────────────────────────
    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function setFreq(freq: number): void {
        let prescaleval: number = 25000000.0;
        prescaleval /= 4096.0;
        prescaleval /= freq;
        prescaleval -= 1.0;

        // ✅ Round ให้ถูกต้อง
        let prescale: number = Math.floor(prescaleval + 0.5);

        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep

        i2cwrite(PCA9685_ADDRESS, MODE1, newmode);     // sleep
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set prescaler
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);     // wake
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1); // restart
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00);
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true;
    }

    // function initPCA9685(): void {
    //     i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
    //     setFreq(50);
    //     for (let idx = 0; idx < 16; idx++) {
    //         setPwm(idx, 0, 0);
    //     }
    //     initialized = true
    // }

    // function setFreq(freq: number): void {
    //     // Constrain the frequency
    //     let prescaleval = 25000000;
    //     prescaleval /= 4096;
    //     prescaleval /= freq;
    //     prescaleval -= 1;
    //     let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
    //     let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
    //     let newmode = (oldmode & 0x7F) | 0x10; // sleep
    //     i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
    //     i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
    //     i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
    //     control.waitMicros(5000);
    //     i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    // }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }

    function setStepper(index: number, dir: boolean): void {
        if (index == 1) {
            if (dir) {
                setPwm(0, STP_CHA_L, STP_CHA_H);
                setPwm(2, STP_CHB_L, STP_CHB_H);
                setPwm(1, STP_CHC_L, STP_CHC_H);
                setPwm(3, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(3, STP_CHA_L, STP_CHA_H);
                setPwm(1, STP_CHB_L, STP_CHB_H);
                setPwm(2, STP_CHC_L, STP_CHC_H);
                setPwm(0, STP_CHD_L, STP_CHD_H);
            }
        } else {
            if (dir) {
                setPwm(4, STP_CHA_L, STP_CHA_H);
                setPwm(6, STP_CHB_L, STP_CHB_H);
                setPwm(5, STP_CHC_L, STP_CHC_H);
                setPwm(7, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(7, STP_CHA_L, STP_CHA_H);
                setPwm(5, STP_CHB_L, STP_CHB_H);
                setPwm(6, STP_CHC_L, STP_CHC_H);
                setPwm(4, STP_CHD_L, STP_CHD_H);
            }
        }
    }

    function stopMotor(index: number) {
        setPwm((index - 1) * 2, 0, 0);
        setPwm((index - 1) * 2 + 1, 0, 0);
    }

    function initBNO055(): void {
        let chipId = i2cread(BNO055_ADDR, 0x00)
        if (chipId != 0xA0) return

        i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x00)
        basic.pause(25)

        i2cwrite(BNO055_ADDR, 0x3F, 0x20)  // soft reset
        basic.pause(650)

        let t = 0
        while (i2cread(BNO055_ADDR, 0x00) != 0xA0 && t < 1000) {
            basic.pause(10); t += 10
        }

        i2cwrite(BNO055_ADDR, 0x3E, 0x00)  // normal power
        basic.pause(10)

        // ✅ 1. ตั้ง Gyro config ก่อน
        i2cwrite(BNO055_ADDR, 0x07, 0x01)  // Page 1
        i2cwrite(BNO055_ADDR, 0x0A, 0x2A)  // ±500dps + LPF 12Hz
        i2cwrite(BNO055_ADDR, 0x07, 0x00)  // Page 0

        // ✅ 2. restore calibration หลัง config
        restoreCalibration()

        // ✅ 3. IMU mode
        i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x08)
        basic.pause(100)  // เพิ่มจาก 30 → 100ms
    }

    // function restoreCalibration(): void {
    //     if (!gg_calSaved) return
    //     let wbuf = pins.createBuffer(23)
    //     wbuf[0] = 0x55
    //     for (let i = 0; i < 22; i++) wbuf[i + 1] = gg_calBuf[i]
    //     pins.i2cWriteBuffer(BNO055_ADDR, wbuf)
    //     basic.pause(10)
    // }
    function restoreCalibration(): void {
        if (!gg_calSaved) return

        // ต้องอยู่ใน CONFIG mode แล้ว (initBNO055 จัดการให้)
        // เขียนทีละ byte ครบ 22 registers
        for (let i = 0; i < 22; i++) {
            i2cwrite(BNO055_ADDR, 0x55 + i, gg_calBuf[i])
        }
        basic.pause(10)
    }
    // function bno055Heading(): number {
    //     pins.i2cWriteNumber(BNO055_ADDR, BNO055_EUL_H_LSB, NumberFormat.UInt8BE)
    //     let buf = pins.i2cReadBuffer(BNO055_ADDR, 2)
    //     return ((buf[0] | (buf[1] << 8)) & 0xFFFF) / 16
    // }
    // 1. แก้ bno055Heading — burst read ถูกต้อง
    function bno055Heading(): number {
        let buf = pins.createBuffer(1)
        buf[0] = BNO055_EUL_H_LSB
        pins.i2cWriteBuffer(BNO055_ADDR, buf, true)
        let data = pins.i2cReadBuffer(BNO055_ADDR, 2)
        let raw = (data[1] << 8) | data[0]
        return (raw & 0xFFFF) / 16.0
    }

    function ggHeadingDiff(target: number, current: number): number {
        let d = target - current
        if (d > 180) d -= 360
        if (d < -180) d += 360
        return d
    }

    function driveLeft(speed: number): void {
        MotorRun(gg_leftMotor, gg_leftMotorDir * speed)
    }

    function driveRight(speed: number): void {
        MotorRun(gg_rightMotor, gg_rightMotorDir * speed)
    }

    // ── Gorilla Go ────────────────────────────────────────────────────────────────

    /**
     * Set up the robot for MotorBit V2.0.
     * Specify left/right motors, encoder pins, wheel sizes, and track width.
     * Call once in "on start".
     * @param leftMotor left drive motor; eg: motorbit.Motors.M1
     * @param leftPin left encoder pin; eg: DigitalPin.P13
     * @param rightMotor right drive motor; eg: motorbit.Motors.M3
     * @param rightPin right encoder pin; eg: DigitalPin.P14
     */
    //% blockId=motorbit_setup_robot
    //% block="Setup Robot|Left Motor %leftMotor Encoder %leftPin|Right Motor %rightMotor Encoder %rightPin|Wheel Dia L (cm) %leftWheelDia R (cm) %rightWheelDia|Track Width (cm) %trackWidth Ticks/Rev %ticksPerRev"
    //% group="Gorilla Go"
    //% weight=100
    //% leftMotor.defl=motorbit.Motors.M1
    //% leftPin.defl=DigitalPin.P2
    //% rightMotor.defl=motorbit.Motors.M2
    //% rightPin.defl=DigitalPin.P0
    //% leftWheelDia.defl=4.2
    //% rightWheelDia.defl=4.2
    //% trackWidth.defl=9.4
    //% ticksPerRev.defl=270
    //% inlineInputMode=external
    export function setupRobot(
        leftMotor: Motors, leftPin: DigitalPin,
        rightMotor: Motors, rightPin: DigitalPin,
        leftWheelDia: number, rightWheelDia: number,
        trackWidth: number, ticksPerRev: number
    ): void {
        if (!initialized) initPCA9685()
        serial.writeLine("PCA9685:" + i2cread(PCA9685_ADDRESS, MODE1) + " BNO055:" + i2cread(BNO055_ADDR, 0x00))
        gg_leftMotor = leftMotor
        gg_rightMotor = rightMotor
        gg_leftWheelDia = leftWheelDia
        gg_rightWheelDia = rightWheelDia
        gg_trackWidth = trackWidth
        gg_ticksPerRev = ticksPerRev
        gg_leftTicks = 0
        gg_rightTicks = 0
        pins.setPull(leftPin, PinPullMode.PullUp)
        pins.setPull(rightPin, PinPullMode.PullUp)
        pins.onPulsed(leftPin, PulseValue.Low, function () { gg_leftTicks += 1 })
        pins.onPulsed(rightPin, PulseValue.Low, function () { gg_rightTicks += 1 })
        initBNO055()
        gg_zeroHeading = bno055Heading()
    }

    //% blockId=motorbit_init_bno055
    //% block="Init BNO055"
    //% group="Gorilla Go"
    //% weight=101
    // export function initIMU(): void {
    //     let A = getDegrees()
    //     // soft recal: CONFIG → IMU, no system reset, no 650 ms wait
    //     i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x00)
    //     basic.pause(25)
    //     i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x08)
    //     basic.pause(30)
    //     gg_zeroHeading = (bno055Heading() - A + 360) % 360
    // }
    // 2. แก้ initIMU — ง่ายและถูกต้อง
    export function initIMU(): void {
        i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x00)
        basic.pause(25)
        i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x08)
        basic.pause(100)  // เพิ่มจาก 30 → 100ms
        gg_zeroHeading = bno055Heading()  // reset zero ตรงๆ
    }
    //% blockId=motorbit_reset_yaw
    //% block="Reset Yaw to 0"
    //% group="Gorilla Go"
    //% weight=92
    export function resetYaw(): void {
        gg_zeroHeading = bno055Heading()
    }

    /**
     * Turn left by a relative angle using tank mode (both wheels move).
     * @param degrees how many degrees to turn left; eg: 90
     * @param speed motor speed 0-255; eg: 70
     */
    //% blockId=gorilla_turn_left_for
    //% block="Turn Left %degrees ° speed %speed"
    //% group="Gorilla Go" weight=98
    //% degrees.min=0 degrees.max=360 degrees.defl=90
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function turnLeftForDegrees(degrees: number, speed: number): void {
        let target = (getDegrees() - degrees + 360) % 360
        headingToDegrees(target, speed)
    }

    /**
     * Turn right by a relative angle using tank mode (both wheels move).
     * @param degrees how many degrees to turn right; eg: 90
     * @param speed motor speed 0-255; eg: 70
     */
    //% blockId=gorilla_turn_right_for
    //% block="Turn Right %degrees ° speed %speed"
    //% group="Gorilla Go" weight=97
    //% degrees.min=0 degrees.max=360 degrees.defl=90
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function turnRightForDegrees(degrees: number, speed: number): void {
        let target = (getDegrees() + degrees) % 360
        headingToDegrees(target, speed)
    }

    /**
     * Turn to face an absolute heading using tank mode (both wheels counter-rotate).
     * @param heading target heading 0-360; eg: 0
     * @param speed motor speed 0-255; eg: 70
     */
    //% blockId=gorilla_heading_to
    //% block="Heading To %heading ° speed %speed"
    //% group="Gorilla Go" weight=96
    //% heading.min=0 heading.max=360 heading.defl=0
    //% speed.min=0 speed.max=255 speed.defl=100
    //% inlineInputMode=inline
    export function headingToDegrees(heading: number, speed: number): void {
        if (!initialized) initPCA9685()
        let turnStart = input.runningTime()
        const TURN_TIMEOUT = 6000   // กันค้างตลอดกาลถ้าเข้าเป้าไม่ได้
        let prevHeading = getDegrees()
        let noChangeMs = 0
        let lastSign = 0

        // Kickstart: ใช้เฉพาะตอน speed ต่ำ (< 100) — กระตุก 200 นาน 100ms
        let kd = ggHeadingDiff(heading, getDegrees())
        if (speed < 100 && Math.abs(kd) > 10) {
            if (kd > 0) { driveLeft(200); driveRight(-200) }
            else { driveLeft(-200); driveRight(200) }
            basic.pause(100)
        }

        // Phase 1: coarse — ramp down from full speed starting at 120° out, exit at 40°
        while (true) {
            if (input.runningTime() - turnStart > TURN_TIMEOUT) break
            let diff = ggHeadingDiff(heading, getDegrees())
            if (Math.abs(diff) <= gg_turnCoarseExit) break
            let turnSpeed = Math.abs(diff) < 120
                ? Math.max(55, Math.round(speed * Math.abs(diff) / 120))
                : speed
            let curSign = diff > 0 ? 1 : -1
            if (lastSign != 0 && curSign != lastSign) {
                MotorStop(gg_leftMotor)
                MotorStop(gg_rightMotor)
                basic.pause(200)
                noChangeMs = 0
            }
            lastSign = curSign
            if (diff > 0) {
                driveLeft(turnSpeed)
                driveRight(-turnSpeed)
            } else {
                driveLeft(-turnSpeed)
                driveRight(turnSpeed)
            }
            basic.pause(20)
            let curHeading = getDegrees()
            if (Math.abs(ggHeadingDiff(curHeading, prevHeading)) < 1) {
                noChangeMs += 20
            } else {
                noChangeMs = 0
            }
            prevHeading = curHeading
            if (noChangeMs >= 400 && Math.abs(diff) > 50) {
                if (diff > 0) {
                    driveLeft(255)
                    driveRight(-255)
                } else {
                    driveLeft(-255)
                    driveRight(255)
                }
                basic.pause(150)
                noChangeMs = 0
            }
        }
        MotorStop(gg_leftMotor)
        MotorStop(gg_rightMotor)
        basic.pause(300)

        // Phase 2: mid — slow continuous tank turn until within 10°
        let p2Prev = getDegrees()
        let p2Stall = 0
        while (true) {
            if (input.runningTime() - turnStart > TURN_TIMEOUT) break
            let diff = ggHeadingDiff(heading, getDegrees())
            if (Math.abs(diff) <= gg_turnMidExit) break
            let ms = gg_turnMidSpeed
            if (diff > 0) {
                driveLeft(ms)
                driveRight(-ms)
            } else {
                driveLeft(-ms)
                driveRight(ms)
            }
            basic.pause(20)
            // Stall kick: สั่งเลี้ยวแล้วองศาไม่ขยับ → กระตุกสั้นๆ (เฉพาะตอนยังไกลเป้า กัน overshoot)
            let p2Cur = getDegrees()
            if (Math.abs(ggHeadingDiff(p2Cur, p2Prev)) < 1) p2Stall += 20
            else p2Stall = 0
            p2Prev = p2Cur
            if (p2Stall >= 200 && Math.abs(diff) > 15) {
                if (diff > 0) { driveLeft(150); driveRight(-150) }
                else { driveLeft(-150); driveRight(150) }
                serial.writeLine("TURN KICK")
                basic.pause(120)
                p2Stall = 0
            }
        }
        MotorStop(gg_leftMotor)
        MotorStop(gg_rightMotor)
        basic.pause(400)

        // Phase 3: fine — short pulses, max 8 attempts or 2000ms, 0.5° resolution
        let fineStart = input.runningTime()
        for (let i = 0; i < 8; i++) {
            if (input.runningTime() - fineStart > 2000) break
            let diff = ggHeadingDiff(heading, getDegrees())
            if (Math.abs(diff) <= gg_turnFineTol) break
            if (diff > 0) {
                driveLeft(90)
                driveRight(-90)
            } else {
                driveLeft(-90)
                driveRight(90)
            }
            basic.pause(25)
            MotorStop(gg_leftMotor)
            MotorStop(gg_rightMotor)
            basic.pause(200)
        }
        MotorStop(gg_leftMotor)
        MotorStop(gg_rightMotor)
    }

    /**
     * Rotate to face an absolute heading using pivot mode (one wheel, other stops).
     * @param heading target heading 0-360; eg: 0
     * @param speed motor speed 0-255; eg: 70
     */
    //% blockId=gorilla_rotate_to
    //% block="Rotate To %heading ° speed %speed"
    //% group="Gorilla Go" weight=95
    //% heading.min=0 heading.max=360 heading.defl=0
    //% speed.min=0 speed.max=255 speed.defl=120
    //% inlineInputMode=inline
    export function rotateToDegrees(heading: number, speed: number): void {
        if (!initialized) initPCA9685()
        let turnStart = input.runningTime()
        const TURN_TIMEOUT = 6000   // กันค้างตลอดกาลถ้าเข้าเป้าไม่ได้
        let prevHeading = getDegrees()
        let noChangeMs = 0
        let useLeft = ggHeadingDiff(heading, getDegrees()) > 0

        function pivotDrive(s: number, diff: number): void {
            let forward = useLeft ? (diff > 0) : (diff < 0)
            let ds = forward ? s : -s
            if (useLeft) { driveLeft(ds); MotorStop(gg_rightMotor) }
            else { MotorStop(gg_leftMotor); driveRight(ds) }
        }

        // Kickstart: ใช้เฉพาะตอน speed ต่ำ (< 100) — กระตุก 200 นาน 100ms
        let kd = ggHeadingDiff(heading, getDegrees())
        if (speed < 100 && Math.abs(kd) > 10) {
            pivotDrive(200, kd)
            basic.pause(100)
        }

        // Phase 1: coarse — ramp down from full speed starting at 120° out, exit at 40°
        while (true) {
            if (input.runningTime() - turnStart > TURN_TIMEOUT) break
            let diff = ggHeadingDiff(heading, getDegrees())
            if (Math.abs(diff) <= gg_turnCoarseExit) break
            let turnSpeed = Math.abs(diff) < 120
                ? Math.max(55, Math.round(speed * Math.abs(diff) / 120))
                : speed
            pivotDrive(turnSpeed, diff)
            basic.pause(20)
            let curHeading = getDegrees()
            if (Math.abs(ggHeadingDiff(curHeading, prevHeading)) < 1) {
                noChangeMs += 20
            } else {
                noChangeMs = 0
            }
            prevHeading = curHeading
            if (noChangeMs >= 400 && Math.abs(diff) > 50) {
                pivotDrive(255, diff)
                basic.pause(150)
                noChangeMs = 0
            }
        }
        MotorStop(gg_leftMotor)
        MotorStop(gg_rightMotor)
        basic.pause(300)

        // Phase 2: mid — slow continuous pivot until within 10°
        let p2Prev = getDegrees()
        let p2Stall = 0
        while (true) {
            if (input.runningTime() - turnStart > TURN_TIMEOUT) break
            let diff = ggHeadingDiff(heading, getDegrees())
            if (Math.abs(diff) <= gg_turnMidExit) break
            pivotDrive(gg_turnMidSpeed, diff)
            basic.pause(20)
            // Stall kick: สั่งหมุนแล้วองศาไม่ขยับ → กระตุกสั้นๆ (เฉพาะตอนยังไกลเป้า กัน overshoot)
            let p2Cur = getDegrees()
            if (Math.abs(ggHeadingDiff(p2Cur, p2Prev)) < 1) p2Stall += 20
            else p2Stall = 0
            p2Prev = p2Cur
            if (p2Stall >= 200 && Math.abs(diff) > 15) {
                pivotDrive(150, diff)
                serial.writeLine("ROTATE KICK")
                basic.pause(120)
                p2Stall = 0
            }
        }
        MotorStop(gg_leftMotor)
        MotorStop(gg_rightMotor)
        basic.pause(400)

        // Phase 3: fine — short pulses, max 8 attempts or 2000ms, 0.5° resolution
        let fineStart = input.runningTime()
        for (let i = 0; i < 8; i++) {
            if (input.runningTime() - fineStart > 2000) break
            let diff = ggHeadingDiff(heading, getDegrees())
            if (Math.abs(diff) <= gg_turnFineTol) break
            pivotDrive(90, diff)
            basic.pause(25)
            MotorStop(gg_leftMotor)
            MotorStop(gg_rightMotor)
            basic.pause(200)
        }
        MotorStop(gg_leftMotor)
        MotorStop(gg_rightMotor)
    }

    /**
     * Drive straight for a given distance (negative = backward).
     * @param distance distance to travel; eg: 30
     * @param unit cm or inch
     * @param speed motor speed 0-255; eg: 150
     */
    //% blockId=gorilla_drive_straight
    //% block="Drive Straight %distance %unit at speed %speed"
    //% group="Gorilla Go" weight=94
    //% distance.defl=30
    //% speed.min=0 speed.max=255 speed.defl=150
    //% inlineInputMode=inline
    export function driveStraight(distance: number, unit: DistanceUnit, speed: number): void {
    if (!initialized) initPCA9685()
    let distCm = unit == DistanceUnit.Inch ? distance * 2.54 : distance
    let leftTarget = Math.round(Math.abs(distCm) / (3.14159 * gg_leftWheelDia) * gg_ticksPerRev)
    let rightTarget = Math.round(Math.abs(distCm) / (3.14159 * gg_rightWheelDia) * gg_ticksPerRev)
    let dir = distCm >= 0 ? 1 : -1
    gg_leftTicks = 0
    gg_rightTicks = 0
    if (leftTarget <= 0 && rightTarget <= 0) return

    // ── ค่าปรับได้: ดึงจากตัวแปร tuning (ตั้งผ่านบล็อก Set Drive Tuning) ──
    const MIN_SPEED = gg_minSpeed   // PWM ต่ำสุดที่ล้อยังหมุน (พ้น deadzone ของ TT motor)
    const KP = gg_kp                // เกน P ของการคุมตรง (ต่อผลต่าง 1 tick)
    const KI = gg_ki                // เกน I สะสมกัน bias ค้าง
    const RAMP_TICKS = gg_rampTicks // ระยะ ramp เข้า/ออก (ticks)
    const CORR_CAP = 40      // จำกัดค่าชดเชยไม่ให้เหวี่ยง (fixed)
    const INTEG_CAP = 400    // กัน integral windup (fixed)
    const STALL_MS = 300     // ไม่มี tick เพิ่มเกินนี้ = ค้าง (fixed)
    const STALL_STEP = 15    // ค้างแล้วค่อยเพิ่ม PWM ทีละขั้น ไม่ slam (fixed)

    // speed ต่ำกว่า deadzone ยกขึ้นเป็น MIN_SPEED (ต่ำกว่านี้ล้อไม่หมุน) — กัน ramp คำนวณติดลบ
    if (speed < MIN_SPEED) {
        serial.writeLine("speed floored to " + MIN_SPEED)
        speed = MIN_SPEED
    }

    // ไม่ต้อง kickstart — MIN_SPEED floor + stall recovery จัดการออกตัวได้เอง

    let integ = 0
    let stallBoost = 0
    let prevSum = 0
    let stallMs = 0
    let loopPrev = input.runningTime()
    let driveStart = loopPrev
    let leftDone = false
    let rightDone = false

    while (true) {
        let now = input.runningTime()
        let dt = now - loopPrev
        loopPrev = now
        if (now - driveStart > 10000) break

        // ถึงเป้าหรือยัง (แยกซ้าย/ขวา) — หยุดล้อที่ถึงก่อน
        if (!leftDone && gg_leftTicks >= leftTarget) { leftDone = true; MotorStop(gg_leftMotor) }
        if (!rightDone && gg_rightTicks >= rightTarget) { rightDone = true; MotorStop(gg_rightMotor) }
        if (leftDone && rightDone) break

        // ── โปรไฟล์ความเร็ว: trapezoid อิงระยะ (ramp เข้า/ชะลอออก) ──
        let avg = (gg_leftTicks + gg_rightTicks) / 2
        let target = (leftTarget + rightTarget) / 2
        let remaining = target - avg
        let accel = avg < RAMP_TICKS ? MIN_SPEED + (speed - MIN_SPEED) * avg / RAMP_TICKS : speed
        let decel = remaining < RAMP_TICKS ? MIN_SPEED + (speed - MIN_SPEED) * remaining / RAMP_TICKS : speed
        let base = Math.max(MIN_SPEED, Math.round(Math.min(accel, decel)))

        // ── stall recovery: ถ้ารอบรวมไม่ขยับ ให้ค่อยเพิ่ม PWM ทีละขั้น (ไม่ slam) ──
        let sum = gg_leftTicks + gg_rightTicks
        if (sum != prevSum) { prevSum = sum; stallMs = 0; stallBoost = 0 }
        else {
            stallMs += dt
            if (stallMs >= STALL_MS) { stallBoost = Math.min(stallBoost + STALL_STEP, 255 - base); stallMs = 0 }
        }
        base = base + stallBoost

        // ── PI คุมตรงด้วยผลต่าง tick ซ้าย-ขวา (symmetric: ลดข้างนำ / เพิ่มข้างตาม) ──
        let err = gg_leftTicks - gg_rightTicks       // + = ซ้ายวิ่งนำ
        integ += err * dt / 100
        if (integ > INTEG_CAP) integ = INTEG_CAP
        if (integ < -INTEG_CAP) integ = -INTEG_CAP
        let corr = KP * err + KI * integ
        if (corr > CORR_CAP) corr = CORR_CAP
        if (corr < -CORR_CAP) corr = -CORR_CAP

        let ls = Math.round(base - corr)
        let rs = Math.round(base + corr)
        if (ls < MIN_SPEED) ls = MIN_SPEED
        if (rs < MIN_SPEED) rs = MIN_SPEED
        if (ls > 255) ls = 255
        if (rs > 255) rs = 255

        if (!leftDone) driveLeft(dir * ls)
        else MotorStop(gg_leftMotor)
        if (!rightDone) driveRight(dir * rs)
        else MotorStop(gg_rightMotor)

        serial.writeLine("base:" + base + " err:" + err + " corr:" + Math.round(corr) + " L:" + gg_leftTicks + " R:" + gg_rightTicks)
        basic.pause(10)
    }
    MotorStop(gg_leftMotor)
    MotorStop(gg_rightMotor)
    serial.writeLine("done LT:" + leftTarget + " L:" + gg_leftTicks + " RT:" + rightTarget + " R:" + gg_rightTicks)
}

    /**
     * ปรับค่าการควบคุม Drive Straight เอง (ถ้าไม่เรียก จะใช้ค่า default: Kp 1.2, Ki 0.05, MinSpeed 45, Ramp 60)
     * @param kp เกน P การคุมตรง (มาก=แก้แรง เสี่ยงแกว่ง); eg: 1.2
     * @param ki เกน I สะสมกัน bias ค้าง; eg: 0.05
     * @param minSpeed PWM ต่ำสุดพ้น deadzone (ออกตัวไม่ขึ้นให้เพิ่ม); eg: 45
     * @param rampTicks ระยะ ramp เข้า/ออก เป็น ticks (มาก=นุ่มขึ้น); eg: 60
     */
    //% blockId=motorbit_set_drive_tuning
    //% block="Set Drive Tuning|Kp %kp Ki %ki|Min Speed %minSpeed Ramp Ticks %rampTicks"
    //% group="Gorilla Go" weight=72
    //% kp.defl=1.2 ki.defl=0.05 minSpeed.defl=45 rampTicks.defl=60
    //% inlineInputMode=external
    export function setDriveTuning(kp: number, ki: number, minSpeed: number, rampTicks: number): void {
        gg_kp = kp
        gg_ki = ki
        gg_minSpeed = minSpeed
        gg_rampTicks = rampTicks
    }

    /**
     * ปรับค่าการเลี้ยว/หมุน เอง (ถ้าไม่เรียก ใช้ default: MidSpeed 65, Coarse 40°, Mid 10°, Fine 0.5°)
     * @param midSpeed speed เฟส 2 หมุนช้าคงที่ (พื้นฝืดให้เพิ่ม); eg: 65
     * @param coarseExit เกณฑ์จบเฟส 1 หยาบ (°); eg: 40
     * @param midExit เกณฑ์จบเฟส 2 กลาง (°); eg: 10
     * @param fineTol ความละเอียดเฟส 3 ละเอียด (°); eg: 0.5
     */
    //% blockId=motorbit_set_turn_tuning
    //% block="Set Turn Tuning|Mid Speed %midSpeed|Coarse %coarseExit ° Mid %midExit ° Fine %fineTol °"
    //% group="Gorilla Go" weight=71
    //% midSpeed.defl=65 coarseExit.defl=40 midExit.defl=10 fineTol.defl=0.5
    //% inlineInputMode=external
    export function setTurnTuning(midSpeed: number, coarseExit: number, midExit: number, fineTol: number): void {
        gg_turnMidSpeed = midSpeed
        gg_turnCoarseExit = coarseExit
        gg_turnMidExit = midExit
        gg_turnFineTol = fineTol
    }

    //% blockId=motorbit_debug_ticks
    //% block="Debug Ticks"
    export function debugTicks(): void {
        serial.writeLine("L:" + gg_leftTicks + " R:" + gg_rightTicks)
    }

    /**
     * Get BNO055 calibration status as a string "SYS:x GYR:x ACC:x MAG:x" (0=uncal, 3=fully cal).
     */
    //% blockId=gorilla_get_calibration
    //% block="Get Calibration Status"
    //% group="Gorilla Go" weight=91
    export function getCalibrationStatus(): string {
        let stat = i2cread(BNO055_ADDR, 0x35)
        let gyr = (stat >> 4) & 0x03
        let acc = (stat >> 2) & 0x03
        return "G:" + gyr + " A:" + acc
    }

    /**
     * Block until gyro is fully calibrated (level 3). Keep robot completely still.
     */
    //% blockId=gorilla_wait_calibration
    //% block="Wait IMU Calibration"
    //% group="Gorilla Go" weight=90
    export function waitForCalibration(): void {
        while (((i2cread(BNO055_ADDR, 0x35) >> 4) & 0x03) < 3) {
            basic.pause(100)
        }
    }

    /**
     * Save BNO055 calibration offsets to RAM. Call after waitForCalibration.
     * Offsets are automatically restored on the next initBNO055 (setupRobot) call.
     * Note: RAM only — lost on power cycle.
     */
    //% blockId=gorilla_save_calibration
    //% block="Save Calibration"
    //% group="Gorilla Go" weight=89
    // export function saveCalibration(): void {
    //     i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x00)
    //     basic.pause(25)
    //     let buf = pins.createBuffer(1)
    //     buf[0] = 0x55
    //     pins.i2cWriteBuffer(BNO055_ADDR, buf, true)
    //     gg_calBuf = pins.i2cReadBuffer(BNO055_ADDR, 22)
    //     gg_calSaved = true
    //     i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x08)
    //     basic.pause(30)
    // }
    export function saveCalibration(): void {
        // ต้องอยู่ใน CONFIG mode
        i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x00)
        basic.pause(25)

        // อ่านทีละ byte ครบ 22 registers
        for (let i = 0; i < 22; i++) {
            gg_calBuf[i] = i2cread(BNO055_ADDR, 0x55 + i)
        }
        gg_calSaved = true

        // กลับ IMU mode
        i2cwrite(BNO055_ADDR, BNO055_OPR_MODE_REG, 0x08)
        basic.pause(100)

        serial.writeLine("Cal saved: " +
            gg_calBuf[0] + "," + gg_calBuf[1] + "," +
            gg_calBuf[12] + "," + gg_calBuf[13])  // debug
    }
    /**
     * Get current heading in degrees 0-360, relative to zero set by setupRobot.
     */
    //% blockId=gorilla_get_degrees
    //% block="Get Degrees (0-360)"
    //% group="Gorilla Go" weight=93
    export function getDegrees(): number {
        return (bno055Heading() - gg_zeroHeading + 360) % 360
    }

    // function setArmAtPercent(percent: number): void {
    //     let leftAngle = gg_leftArmOpenAngle + (gg_leftArmCloseAngle - gg_leftArmOpenAngle) * percent / 100
    //     let rightAngle = gg_rightArmOpenAngle + (gg_rightArmCloseAngle - gg_rightArmOpenAngle) * percent / 100
    //     Servo(gg_leftArmServo, leftAngle)
    //     Servo(gg_rightArmServo, rightAngle)
    //     gg_currentArmPercent = percent
    // }
    function setArmAtPercent(percent: number): void {
        let leftAngle = gg_leftArmOpenAngle + (gg_leftArmCloseAngle - gg_leftArmOpenAngle) * percent / 100
        let rightAngle = gg_rightArmOpenAngle + (gg_rightArmCloseAngle - gg_rightArmOpenAngle) * percent / 100
        Servo(gg_leftArmServo, Math.round(leftAngle))
        Servo(gg_rightArmServo, Math.round(rightAngle))
        gg_currentArmPercent = percent
        gg_currentLeftArmPercent = percent
        gg_currentRightArmPercent = percent
    }

    function setLeftArmAtPercent(percent: number): void {
        let leftAngle = gg_leftArmOpenAngle + (gg_leftArmCloseAngle - gg_leftArmOpenAngle) * percent / 100
        Servo(gg_leftArmServo, Math.round(leftAngle))
        gg_currentLeftArmPercent = percent
    }

    function setRightArmAtPercent(percent: number): void {
        let rightAngle = gg_rightArmOpenAngle + (gg_rightArmCloseAngle - gg_rightArmOpenAngle) * percent / 100
        Servo(gg_rightArmServo, Math.round(rightAngle))
        gg_currentRightArmPercent = percent
    }

    function moveLeftArmToPercent(targetPercent: number, speed: number): void {
        if (targetPercent < 0) targetPercent = 0
        if (targetPercent > 100) targetPercent = 100
        let startPercent = gg_currentLeftArmPercent
        let diff = targetPercent - startPercent
        serial.writeLine("moveLeft " + Math.round(startPercent) + " -> " + targetPercent)
        if (diff == 0) return

        const STEPS = 100
        let delay = 4 * (10 - speed)

        for (let i = 1; i <= STEPS; i++) {
            let p = startPercent + diff * i / STEPS
            setLeftArmAtPercent(p)
            basic.pause(delay)
        }
        setLeftArmAtPercent(targetPercent)
    }

    function moveRightArmToPercent(targetPercent: number, speed: number): void {
        if (targetPercent < 0) targetPercent = 0
        if (targetPercent > 100) targetPercent = 100
        let startPercent = gg_currentRightArmPercent
        let diff = targetPercent - startPercent
        serial.writeLine("moveRight " + Math.round(startPercent) + " -> " + targetPercent)
        if (diff == 0) return

        const STEPS = 100
        let delay = 4 * (10 - speed)

        for (let i = 1; i <= STEPS; i++) {
            let p = startPercent + diff * i / STEPS
            setRightArmAtPercent(p)
            basic.pause(delay)
        }
        setRightArmAtPercent(targetPercent)
    }

    // function moveArmToPercent(targetPercent: number, speed: number): void {
    //     if (targetPercent < 0) targetPercent = 0
    //     if (targetPercent > 100) targetPercent = 100
    //     let startPercent = gg_currentArmPercent
    //     let steps = Math.abs(targetPercent - startPercent)
    //     if (steps < 1) {
    //         setArmAtPercent(targetPercent)
    //         return
    //     }
    //     let dir = targetPercent > startPercent ? 1 : -1
    //     for (let i = 1; i <= steps; i++) {
    //         basic.pause(4 * (10 - speed))
    //         setArmAtPercent(startPercent + dir * i)
    //     }
    //     setArmAtPercent(targetPercent)
    // }
    function moveArmToPercent(targetPercent: number, speed: number): void {
        if (targetPercent < 0) targetPercent = 0
        if (targetPercent > 100) targetPercent = 100

        // เริ่มจากตำแหน่งจริงของแต่ละแขน (อาจต่างกันถ้าเพิ่งสั่งทีละข้าง)
        let startLeft = gg_currentLeftArmPercent
        let startRight = gg_currentRightArmPercent
        let diffLeft = targetPercent - startLeft
        let diffRight = targetPercent - startRight
        serial.writeLine("moveArm L:" + Math.round(startLeft) + " R:" + Math.round(startRight) + " -> " + targetPercent)

        const STEPS = 100  // แบ่งการเคลื่อนที่เป็น 100 สเต็ปเสมอ ไม่ว่าระยะจะสั้นหรือยาว
        let delay = 4 * (10 - speed)  // delay คงที่ต่อสเต็ป ไม่แปรผันตามระยะทาง

        for (let i = 1; i <= STEPS; i++) {
            setLeftArmAtPercent(startLeft + diffLeft * i / STEPS)
            setRightArmAtPercent(startRight + diffRight * i / STEPS)
            basic.pause(delay)
        }
        // กันเศษปัดเข้าตำแหน่งจริงเป๊ะๆ ตอนจบ
        setLeftArmAtPercent(targetPercent)
        setRightArmAtPercent(targetPercent)
        gg_currentArmPercent = targetPercent
    }

    /**
     * Setup arm ที่ใช้ Servo 2 ตัว (ซ้าย/ขวา) แยกอิสระจากกัน
     * เมื่อเปิดแขนสุด: Left Arm = leftOpenAngle, Right Arm = rightOpenAngle
     * เมื่อปิดแขนสุด: Left Arm = leftCloseAngle, Right Arm = rightCloseAngle
     * @param leftArmServo servo แขนซ้าย; eg: motorbit.Servos.S2
     * @param leftOpenAngle มุมแขนซ้ายตอนเปิดสุด; eg: 0
     * @param leftCloseAngle มุมแขนซ้ายตอนปิดสุด; eg: 210
     * @param rightArmServo servo แขนขวา; eg: motorbit.Servos.S1
     * @param rightOpenAngle มุมแขนขวาตอนเปิดสุด; eg: 210
     * @param rightCloseAngle มุมแขนขวาตอนปิดสุด; eg: 0
     */
    //% blockId=motorbit_setup_arm
    //% block="Setup Arm|Left Arm Servo %leftArmServo open %leftOpenAngle° close %leftCloseAngle°|Right Arm Servo %rightArmServo open %rightOpenAngle° close %rightCloseAngle°"
    //% group="Gorilla Go"
    //% weight=99
    //% leftArmServo.defl=motorbit.Servos.S2
    //% rightArmServo.defl=motorbit.Servos.S1
    //% leftOpenAngle.min=0 leftOpenAngle.max=210 leftOpenAngle.defl=210
    //% leftCloseAngle.min=0 leftCloseAngle.max=210 leftCloseAngle.defl=0
    //% rightOpenAngle.min=0 rightOpenAngle.max=210 rightOpenAngle.defl=0
    //% rightCloseAngle.min=0 rightCloseAngle.max=210 rightCloseAngle.defl=210
    //% inlineInputMode=external
    export function setupArm(
        leftArmServo: Servos, leftOpenAngle: number, leftCloseAngle: number,
        rightArmServo: Servos, rightOpenAngle: number, rightCloseAngle: number
    ): void {
        gg_leftArmServo = leftArmServo
        gg_rightArmServo = rightArmServo
        gg_leftArmOpenAngle = leftOpenAngle
        gg_leftArmCloseAngle = leftCloseAngle
        gg_rightArmOpenAngle = rightOpenAngle
        gg_rightArmCloseAngle = rightCloseAngle
        setArmAtPercent(0)   // ตั้งค่าเริ่มต้นให้เปิดสุด
    }

    /**
     * เปิดแขนสุด (Arm)
     * @param speed ความเร็วในการเปิด 1-10; eg: 5
     */
    //% blockId=motorbit_open_arm
    //% block="Open Arm speed %speed"
    //% group="Gorilla Go"
    //% weight=77
    //% speed.min=1 speed.max=10 speed.defl=5
    export function openArm(speed: number): void {
        moveArmToPercent(0, speed)
    }

    /**
     * ปิดแขน (Arm) ตามเปอร์เซ็นต์ที่ต้องการ (0 = เปิดสุด, 100 = ปิดสุด)
     * @param percent เปอร์เซ็นต์การปิด 0-100; eg: 100
     * @param speed ความเร็วในการปิด 1-10; eg: 5
     */
    //% blockId=motorbit_close_arm
    //% block="Close Arm %percent \\% speed %speed"
    //% group="Gorilla Go"
    //% weight=76
    //% percent.min=0 percent.max=100 percent.defl=100
    //% speed.min=1 speed.max=10 speed.defl=5
    export function closeArm(percent: number, speed: number): void {
        moveArmToPercent(percent, speed)
    }

    /**
     * ปิดเฉพาะแขนซ้าย (Arm) ตามเปอร์เซ็นต์ที่ต้องการ โดยเริ่มจากค่าปัจจุบัน
     * (0 = เปิดสุด, 100 = ปิดสุด)
     * @param percent เปอร์เซ็นต์การปิดแขนซ้าย 0-100; eg: 100
     * @param speed ความเร็วในการปิด 1-10; eg: 5
     */
    //% blockId=motorbit_close_left_arm
    //% block="Close Left Arm %percent \\% speed %speed"
    //% group="Gorilla Go"
    //% weight=74
    //% percent.min=0 percent.max=100 percent.defl=100
    //% speed.min=1 speed.max=10 speed.defl=5
    export function CloseLeftArm(percent: number, speed: number): void {
        moveLeftArmToPercent(percent, speed)
    }

    /**
     * ปิดเฉพาะแขนขวา (Arm) ตามเปอร์เซ็นต์ที่ต้องการ โดยเริ่มจากค่าปัจจุบัน
     * (0 = เปิดสุด, 100 = ปิดสุด)
     * @param percent เปอร์เซ็นต์การปิดแขนขวา 0-100; eg: 100
     * @param speed ความเร็วในการปิด 1-10; eg: 5
     */
    //% blockId=motorbit_close_right_arm
    //% block="Close Right Arm %percent \\% speed %speed"
    //% group="Gorilla Go"
    //% weight=73
    //% percent.min=0 percent.max=100 percent.defl=100
    //% speed.min=1 speed.max=10 speed.defl=5
    export function CloseRightArm(percent: number, speed: number): void {
        moveRightArmToPercent(percent, speed)
    }

    /**
     * ดึงค่าเปอร์เซ็นต์การปิดปัจจุบันของแขน (Arm)
     * 0 = เปิดสุด, 100 = ปิดสุด
     */
    //% blockId=motorbit_get_current_arm_percent
    //% block="Current Arm Percent"
    //% group="Gorilla Go" weight=75
    export function getCurrentArmPercent(): number {
        return gg_currentArmPercent
    }

    // ── Motor ─────────────────────────────────────────────────────────────────────

    //% blockId=motorbit_motor_run block="Motor|%index|speed %speed"
    //% group="Motor" weight=86
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRun(index: Motors, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        speed = speed * 16; // map 255 to 4096
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= -4096) {
            speed = -4095
        }
        if (index > 4 || index <= 0)
            return
        let pp = (index - 1) * 2
        let pn = (index - 1) * 2 + 1
        if (speed >= 0) {
            setPwm(pp, 0, speed)
            setPwm(pn, 0, 0)
        } else {
            setPwm(pp, 0, 0)
            setPwm(pn, 0, -speed)
        }
    }

    //% blockId=motorbit_stop block="Motor Stop|%index|"
    //% group="Motor" weight=82
    export function MotorStop(index: Motors): void {
        MotorRun(index, 0);
    }

    //% blockId=motorbit_stop_all block="Motor Stop All"
    //% group="Motor" weight=81
    //% blockGap=50
    export function MotorStopAll(): void {
        if (!initialized) {
            initPCA9685()
        }
        for (let idx = 1; idx <= 4; idx++) {
            stopMotor(idx);
        }
    }

    /**
     * Execute single motors with delay
     * @param index Motor Index; eg: A01A02, B01B02, A03A04, B03B04
     * @param speed [-255-255] speed of motor; eg: 150, -150
     * @param delay seconde delay to stop; eg: 1
    */
    //% blockId=motorbit_motor_rundelay block="Motor|%index|speed %speed|delay %delay|s"
    //% group="Motor" weight=85
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRunDelay(index: Motors, speed: number, delay: number): void {
        MotorRun(index, speed);
        basic.pause(delay * 1000);
        MotorRun(index, 0);
    }

    /**
     * Execute two motors at the same time
     * @param motor1 First Motor; eg: A01A02, B01B02
     * @param speed1 [-255-255] speed of motor; eg: 150, -150
     * @param motor2 Second Motor; eg: A03A04, B03B04
     * @param speed2 [-255-255] speed of motor; eg: 150, -150
    */
    //% blockId=motorbit_motor_dual block="Motor|%motor1|speed %speed1|%motor2|speed %speed2"
    //% group="Motor" weight=84
    //% inlineInputMode=inline
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRunDual(motor1: Motors, speed1: number, motor2: Motors, speed2: number): void {
        MotorRun(motor1, speed1);
        MotorRun(motor2, speed2);
    }

    /**
     * Execute two motors at the same time
     * @param motor1 First Motor; eg: A01A02, B01B02
     * @param speed1 [-255-255] speed of motor; eg: 150, -150
     * @param motor2 Second Motor; eg: A03A04, B03B04
     * @param speed2 [-255-255] speed of motor; eg: 150, -150
    */
    //% blockId=motorbit_motor_dualDelay block="Motor|%motor1|speed %speed1|%motor2|speed %speed2|delay %delay|s "
    //% group="Motor" weight=83
    //% inlineInputMode=inline
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=5
    export function MotorRunDualDelay(motor1: Motors, speed1: number, motor2: Motors, speed2: number, delay: number): void {
        MotorRun(motor1, speed1);
        MotorRun(motor2, speed2);
        basic.pause(delay * 1000);
        MotorRun(motor1, 0);
        MotorRun(motor2, 0);
    }

    // ── Servo ─────────────────────────────────────────────────────────────────────

    /**
     * Servo Execute
     * @param index Servo Channel; eg: S1
     * @param degree [0-180] degree of servo; eg: 0, 90, 180
    */
    //% blockId=motorbit_servo block="Servo|%index|degree|%degree"
    //% group="Servo" weight=100
    //% degree.defl=90
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function Servo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        let v_us = (degree * 1800 / 180 + 600) // 0.6 ~ 2.4
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }
    // export function Servo(index: Servos, degree: number): void {
    //     if (!initialized) initPCA9685()
    //     let v_us = Math.round(degree * 1800 / 180 + 600)
    //     let value = Math.round(v_us * 4096 / 20000)
    //     setPwm(index + 7, 0, value)
    // }

    /**
     * Servo Execute
     * @param index Servo Channel; eg: S1
     * @param degree1 [0-180] degree of servo; eg: 0, 90, 180
     * @param degree2 [0-180] degree of servo; eg: 0, 90, 180
     * @param speed [1-10] speed of servo; eg: 1, 10
    */
    //% blockId=motorbit_servospeed block="Servo|%index|degree start %degree1|end %degree2|speed %speed"
    //% group="Servo" weight=96
    //% degree1.min=0 degree1.max=180
    //% degree2.min=0 degree2.max=180
    //% speed.min=1 speed.max=10
    //% inlineInputMode=inline
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function Servospeed(index: Servos, degree1: number, degree2: number, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        if (degree1 > degree2) {
            for (let i = degree1; i > degree2; i--) {
                let v_us = (i * 1800 / 180 + 600) // 0.6 ~ 2.4
                let value = v_us * 4096 / 20000
                basic.pause(4 * (10 - speed));
                setPwm(index + 7, 0, value)
            }
        } else {
            for (let i = degree1; i < degree2; i++) {
                let v_us = (i * 1800 / 180 + 600) // 0.6 ~ 2.4
                let value = v_us * 4096 / 20000
                basic.pause(4 * (10 - speed));
                setPwm(index + 7, 0, value)
            }
        }
    }

    // ── GeekServo ─────────────────────────────────────────────────────────────────

    /**
     * Geek Servo
     * @param index Servo Channel; eg: S1
     * @param degree [-45-225] degree of servo; eg: -45, 90, 225
    */
    //% blockId=motorbit_gservo block="Geek Servo|%index|degree %degree=protractorPicker"
    //% group="GeekServo" weight=96
    //% blockGap=50
    //% degree.defl=90
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function EM_GeekServo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        let v_us = ((degree - 90) * 20 / 3 + 1500) // 0.6 ~ 2.4
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    /**
     * GeekServo2KG
     * @param index Servo Channel; eg: S1
     * @param degree [0-360] degree of servo; eg: 0, 180, 360
    */
    //% blockId=motorbit_gservo2kg block="GeekServo2KG|%index|degree %degree"
    //% group="GeekServo" weight=95
    //% blockGap=50
    //% degree.min=0 degree.max=360
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function EM_GeekServo2KG(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let v_us = (Math.floor((degree) * 2000 / 350) + 500) //fixed
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    /**
     * GeekServo5KG
     * @param index Servo Channel; eg: S1
     * @param degree [0-360] degree of servo; eg: 0, 180, 360
    */
    //% blockId=motorbit_gservo5kg block="GeekServo5KG|%index|degree %degree"
    //% group="GeekServo" weight=94
    //% degree.min=0 degree.max=360
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function EM_GeekServo5KG(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        const minInput = 0;
        const maxInput = 355;
        const minOutput = 500;
        const maxOutput = 2500;
        const v_us = ((degree - minInput) / (maxInput - minInput)) * (maxOutput - minOutput) + minOutput;
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    //% blockId=motorbit_gservo5kg_motor block="GeekServo5KG_MotorEN|%index|speed %speed"
    //% group="GeekServo" weight=93
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function EM_GeekServo5KG_Motor(index: Servos, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        const minInput = -255;
        const maxInput = 255;
        const minOutput = 5000;
        const maxOutput = 3000;
        const v_us = ((speed - minInput) / (maxInput - minInput)) * (maxOutput - minOutput) + minOutput;
        let value = v_us * 4096 / 20000
        setPwm(index + 7, 0, value)
    }

    // ── Stepper Motor ─────────────────────────────────────────────────────────────

    //% blockId=motorbit_stepper_degree block="Stepper 28BYJ-48|%index|degree %degree"
    //% group="Stepper Motor" weight=91
    export function StepperDegree(index: Steppers, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        setStepper(index, degree > 0);
        degree = Math.abs(degree);
        basic.pause(10240 * degree / 360);
        MotorStopAll()
    }

    //% blockId=motorbit_stepper_turn block="Stepper 28BYJ-48|%index|turn %turn"
    //% group="Stepper Motor" weight=90
    export function StepperTurn(index: Steppers, turn: Turns): void {
        let degree = turn;
        StepperDegree(index, degree);
    }

    //% blockId=motorbit_stepper_dual block="Dual Stepper(Degree) |STPM1_2 %degree1| STPM3_4 %degree2"
    //% group="Stepper Motor" weight=89
    export function StepperDual(degree1: number, degree2: number): void {
        if (!initialized) {
            initPCA9685()
        }
        setStepper(1, degree1 > 0);
        setStepper(2, degree2 > 0);
        degree1 = Math.abs(degree1);
        degree2 = Math.abs(degree2);
        basic.pause(10240 * Math.min(degree1, degree2) / 360);
        if (degree1 > degree2) {
            stopMotor(3); stopMotor(4);
            basic.pause(10240 * (degree1 - degree2) / 360);
        } else {
            stopMotor(1); stopMotor(2);
            basic.pause(10240 * (degree2 - degree1) / 360);
        }
        MotorStopAll()
    }

    /**
     * Stepper Car move forward
     * @param distance Distance to move in cm; eg: 10, 20
     * @param diameter diameter of wheel in mm; eg: 48
    */
    //% blockId=motorbit_stpcar_move block="Car Forward|Distance(cm) %distance|Wheel Diameter(mm) %diameter"
    //% group="Stepper Motor" weight=88
    export function StpCarMove(distance: number, diameter: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let delay = 10240 * 10 * distance / 3 / diameter; // use 3 instead of pi
        setStepper(1, delay > 0);
        setStepper(2, delay > 0);
        delay = Math.abs(delay);
        basic.pause(delay);
        MotorStopAll()
    }

    /**
     * Stepper Car turn by degree
     * @param turn Degree to turn; eg: 90, 180, 360
     * @param diameter diameter of wheel in mm; eg: 48
     * @param track track width of car; eg: 125
    */
    //% blockId=motorbit_stpcar_turn block="Car Turn|Degree %turn|Wheel Diameter(mm) %diameter|Track(mm) %track"
    //% weight=87
    //% group="Stepper Motor" blockGap=50
    export function StpCarTurn(turn: number, diameter: number, track: number): void {
        if (!initialized) {
            initPCA9685()
        }
        let delay = 10240 * turn * track / 360 / diameter;
        setStepper(1, delay < 0);
        setStepper(2, delay > 0);
        delay = Math.abs(delay);
        basic.pause(delay);
        MotorStopAll()
    }

    // ── RUS-04 ────────────────────────────────────────────────────────────────────

    //% blockId="motorbit_rus04" block="On-board Ultrasonic part %index show color %rgb effect %effect"
    //% group="RUS-04" weight=78
    export function motorbit_rus04(index: RgbUltrasonics, rgb: RgbColors, effect: ColorEffect): void {
        sensors.board_rus04_rgb(DigitalPin.P16, 4, index, rgb, effect);
    }

    //% blockId=Ultrasonic_reading_distance block="On-board Ultrasonic reading distance"
    //% group="RUS-04" weight=77
    export function Ultrasonic_reading_distance(): number {
        return sensors.Ultrasonic(DigitalPin.P2);
    }

    // ── RGB ───────────────────────────────────────────────────────────────────────

    //% blockId=Setting_the_on_board_lights block="Setting the on-board lights %index color %rgb"
    //% group="RGB" weight=76
    export function Setting_the_on_board_lights(index: Offset, rgb: RgbColors): void {
        sensors.board_rus04_rgb(DigitalPin.P16, index, 0, rgb, rgb_ColorEffect.None);
    }

    //% blockId=close_the_on_board_lights block="close the on-board lights %index color"
    //% group="RGB" weight=75
    export function close_the_on_board_lights(index: Offset): void {
        sensors.board_rus04_rgb(DigitalPin.P16, index, 0, RgbColors.Black, rgb_ColorEffect.None);
    }

    //% blockId=close_all_the_on_board_lights block="close all the on-board lights"
    //% group="RGB" weight=74
    export function close_all_the_on_board_lights(): void {
        sensors.board_rus04_rgb(DigitalPin.P16, 0, 0, RgbColors.Black, rgb_ColorEffect.None);
        sensors.board_rus04_rgb(DigitalPin.P16, 1, 0, RgbColors.Black, rgb_ColorEffect.None);
        sensors.board_rus04_rgb(DigitalPin.P16, 2, 0, RgbColors.Black, rgb_ColorEffect.None);
        sensors.board_rus04_rgb(DigitalPin.P16, 3, 0, RgbColors.Black, rgb_ColorEffect.None);
    }



    // basic.forever(function () {
    //     let raw = bno055Heading()
    //     let zero = gg_zeroHeading
    //     let diff = (raw - zero + 360) % 360
    //     serial.writeLine("raw:" + raw + " zero:" + zero + " diff:" + diff)
    //     basic.pause(100)
    // })

}
