// tests go here; this will not be compiled when this package is used as a library

motorbit.setupRobot(
    motorbit.Motors.M1, DigitalPin.P2,
    motorbit.Motors.M2, DigitalPin.P0,
    4.7, 4.7, 8.8, 270
)
// แขน 2 servo: ซ้าย S2, ขวา S1
motorbit.setupArm(
    motorbit.Servos.S2, 210, 0,
    motorbit.Servos.S1, 0, 210
)
// จูน Drive Straight (ค่า default — ปรับเลขได้ตามพื้น/หุ่นจริง)
// Kp=แก้ตรง, Ki=กัน bias, MinSpeed=พ้น deadzone, RampTicks=นุ่มตอนเร่ง/เบรก
motorbit.setDriveTuning(1.2, 0.05, 45, 60)
// จูนเลี้ยว/หมุน (ค่า default)
// MidSpeed=speed เฟส 2 (พื้นฝืดให้เพิ่ม), Coarse/Mid/Fine=เกณฑ์องศาแต่ละเฟส
motorbit.setTurnTuning(65, 40, 10, 0.5)

// ── Button A: RAW motor + encoder test (แยกฮาร์ดแวร์ออกจากตรรกะ) ──
// สปินมอเตอร์ตรง ๆ 1 วินาที ถ้าไม่ขยับ = ปัญหาไฟ/สายมอเตอร์ ไม่ใช่โค้ด
// input.onButtonPressed(Button.A, function () {
//     motorbit.resetYaw()   // set zero ก่อนเริ่มทดสอบ
//     basic.showString("R")
//     serial.writeLine("RAW motor test: M1=left M2=right speed 150")
//     motorbit.MotorRun(motorbit.Motors.M1, 150)   // ล้อซ้าย
//     motorbit.MotorRun(motorbit.Motors.M2, 150)   // ล้อขวา
//     basic.pause(1000)
//     motorbit.MotorStopAll()
//     motorbit.debugTicks()   // ดูว่า encoder นับรอบไหม (ควร > 0 ถ้าล้อหมุน)
//     basic.showIcon(IconNames.Yes)
// })

// ── Button A (ค้าง) → หรือแยกทดสอบ Drive Straight ที่ speed ต่ำ ──
// เปิดใช้เมื่อ RAW test ผ่านแล้ว
input.onButtonPressed(Button.A, function () {
    basic.showString("D")
    motorbit.resetYaw()   // set zero ก่อนเริ่มทดสอบ
    motorbit.driveStraight(30, motorbit.DistanceUnit.CM, 60)
    basic.pause(600)
    motorbit.driveStraight(-30, motorbit.DistanceUnit.CM, 60)
    basic.showIcon(IconNames.Yes)
})

// ── Button B: ทดสอบการเลี้ยว (tank) + หมุน (pivot) ──
input.onButtonPressed(Button.B, function () {
    basic.showString("T")
    motorbit.turnLeftForDegrees(90, 100)
    basic.pause(500)
    motorbit.turnRightForDegrees(90, 100)
    basic.pause(500)
    // กลับมาหันหน้าที่ 0 องศาด้วย pivot
    motorbit.rotateToDegrees(0, 100)
    serial.writeLine("heading=" + Math.round(motorbit.getDegrees()))
    basic.showIcon(IconNames.Yes)
})

// ── Button A+B: ทดสอบแขน (รวม + แยกซ้าย/ขวา) ──
input.onButtonPressed(Button.AB, function () {
    basic.showString("A")
    // ปิดพร้อมกัน แล้วเปิด
    motorbit.closeArm(100, 5)
    basic.pause(500)
    motorbit.openArm(5)
    basic.pause(500)
    // ทำให้สองข้างไม่เท่ากัน: ปิดซ้ายสุด ขวาครึ่งเดียว
    motorbit.CloseLeftArm(100, 5)
    motorbit.CloseRightArm(50, 5)
    basic.pause(500)
    // เปิดพร้อมกัน — ต้องกลับมาเปิดสุดทั้งคู่แม้เริ่มจากตำแหน่งไม่เท่ากัน
    motorbit.openArm(5)
    serial.writeLine("armPct=" + motorbit.getCurrentArmPercent())
    basic.showIcon(IconNames.Yes)
})

basic.showString("RDY")
