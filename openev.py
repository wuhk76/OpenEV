import gpiozero
import time
import math
import keyboard
import board
import busio
import adafruit_mpu6050
import subprocess
subprocess.run('killall pigpiod', shell = True)
time.sleep(0.5)
subprocess.run('pigpiod', shell = True)
import pigpio
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c, address = 0x68)
class Motor:
    def __init__(self):
        self.pi = pigpio.pi()
        self.motors = {'outputs': (20, 21), 'signs': (23, 24)}
        self.freq = 1000
        for a, b in self.motors.values():
            self.pi.set_mode(a, pigpio.OUTPUT)
            self.pi.set_mode(b, pigpio.OUTPUT)
    def setpins(self, outputs, signs):
        self.motors['outputs'] = tuple(outputs)
        self.motors['signs'] = tuple(signs)
    def write(self, throttle, yaw):
        throttle = max(-1, min(1, throttle))
        yaw = max(-1, min(1, yaw))
        throttle = throttle / 2
        yaw = yaw / 2
        dutya = max(-255, min(255, int((255 * (throttle + yaw)) // 1)))
        dutyb = max(-255, min(255, int((255 * (throttle - yaw)) // 1)))
        for a, b in self.motors.values():
            self.pi.write(a, 0)
            self.pi.write(b, 0)
        time.sleep(0.01)
        self.pi.write(self.motors['signs'][0], 1 if dutya >= 0 else 0)
        self.pi.write(self.motors['signs'][1], 1 if dutyb >= 0 else 0)
        self.pi.set_PWM_frequency(self.motors['outputs'][0], self.freq)
        self.pi.set_PWM_frequency(self.motors['outputs'][1], self.freq)
        self.pi.set_PWM_dutycycle(self.motors['outputs'][0], abs(dutya))
        self.pi.set_PWM_dutycycle(self.motors['outputs'][1], abs(dutyb))
    def kill(self):
        for a, b in self.motors.values():
            self.pi.write(a, 0)
            self.pi.write(b, 0)
        self.pi.stop()
class Accel:
    def __init__(self, mpu):
        self.mpu = mpu
        self.origin = 0, 0, 0
        self.vel = [0, (0, 0)]
        self.pos = [0, (0, 0)]
    def reset(self):
        self.origin = 0, 0, 0
        self.vel = [0, (0, 0)]
        self.pos = [0, (0, 0)]
    def getacc(self):
        origin = self.origin
        acc = self.mpu.acceleration
        return acc[0] - origin[0], acc[1] - origin[1], acc[2] - origin[2]
    def update(self):
        offset = []
        for j in range(100):
            offset.append(self.mpu.acceleration)
        offsetx = sum([i[0] for i in offset]) / len(offset)
        offsety = sum([i[1] for i in offset]) / len(offset)
        offsetz = sum([i[2] for i in offset]) / len(offset)
        self.origin = offsetx, offsety, offsetz
    def resetintegral(self):
        self.vel = [0, (time.perf_counter(), 0)]
        self.pos = [0, (time.perf_counter(), 0)]
    def velocity(self, point):
        dx = point[0] - self.vel[1][0]
        mean = (point[1] + self.vel[1][1]) / 2
        self.vel[0] += dx * mean
        self.vel[1] = self.vel[1][0] + dx, point[1]
        return self.vel[1][0], self.vel[0]
    def position(self, point):
        dx = point[0] - self.pos[1][0]
        mean = (point[1] + self.pos[1][1]) / 2
        self.pos[0] += dx * mean
        self.pos[1] = self.pos[1][0] + dx, point[1]
        return self.pos[1][0], self.pos[0]
class Gyro:
    def __init__(self, mpu):
        self.mpu = mpu
        self.origin = 0, 0, 0
        self.angle = [0, (0, 0)]
    def reset(self):
        self.origin = 0, 0, 0
    def getgyro(self):
        origin = self.origin
        gyro = self.mpu.gyro
        return gyro[0] - origin[0], gyro[1] - origin[1], gyro[2] - origin[2]
    def update(self):
        offset = []
        for j in range(100):
            offset.append(self.mpu.gyro)
        offsetx = sum([i[0] for i in offset]) / len(offset)
        offsety = sum([i[1] for i in offset]) / len(offset)
        offsetz = sum([i[2] for i in offset]) / len(offset)
        self.origin = offsetx, offsety, offsetz
    def theta(self, point):
        dx = point[0] - self.angle[1][0]
        mean = (point[1] + self.angle[1][1]) / 2
        self.angle[0] += dx * mean
        self.angle[1] = self.angle[1][0] + dx, point[1]
        return self.angle[1][0], self.angle[0]
    def resetintegral(self):
        self.angle = [0, (time.perf_counter(), 0)]
class Hcsr04:
    def __init__(self):
        self.sensor = gpiozero.DistanceSensor(echo = 16, trigger = 12)
    def getdist(self):
        distance = 0.0
        for j in range(10):
            distance += self.sensor.distance
        return distance * 10
motor = Motor()
accel = Accel(mpu)
gyro = Gyro(mpu)
hcsr04 = Hcsr04()
def forward(sec, speed = 1, turn = 0.5):
    gyro.resetintegral()
    gyro.update()
    tstart = time.perf_counter()
    while time.perf_counter() - tstart < sec:
        t = time.perf_counter()
        angvel = round(gyro.getgyro()[2], 2)
        angle = gyro.theta((t, angvel))[1]
        if angle < -0.05:
            motor.write(speed, -turn)
        elif angle > 0.05:
            motor.write(speed, turn)
        else:
            motor.write(speed, 0)
    motor.write(0, 0)
def back(sec, speed = 1, turn = 0.5):
    gyro.resetintegral()
    gyro.update()
    tstart = time.perf_counter()
    while time.perf_counter() - tstart < sec:
        t = time.perf_counter()
        angvel = round(gyro.getgyro()[2], 2)
        angle = gyro.theta((t, angvel))[1]
        if angle < -0.05:
            motor.write(-speed, -turn)
        elif angle > 0.05:
            motor.write(-speed, turn)
        else:
            motor.write(-speed, 0)
    motor.write(0, 0)
def rotate(degrees, rate = 0.2, timeout = 2):
    counter = 0
    gyro.update()
    gyro.resetintegral()
    t = 0
    tstart = time.perf_counter()
    while t - tstart < timeout:
        t = time.perf_counter()
        avel = gyro.getgyro()[2]
        angle = gyro.theta((t, avel))[1]
        angle = math.degrees(angle)
        yaw = rate * (angle - degrees)
        motor.write(0, yaw)
        if abs(yaw) < abs(rate / 2):
            counter += 1
            if counter > 100:
                break
def smartmove(speed = 1, turn = 0.5, margin = 20):
    gyro.resetintegral()
    gyro.update()
    dist = 100
    while dist > margin:
        t = time.perf_counter()
        dist = hcsr04.getdist()
        angvel = round(gyro.getgyro()[2], 2)
        angle = gyro.theta((t, angvel))[1]
        if angle < -0.05:
            motor.write(speed, -turn)
        elif angle > 0.05:
            motor.write(speed, turn)
        else:
            motor.write(speed, 0)
    motor.write(0, 0)
def bluctl():
    prevthrottle = 0.0
    prevyaw = 0.0
    throttle = 0.0
    yaw = 0.0
    while True:
        try:
            yaw = 0.0
            for j in range(10):
                if keyboard.is_pressed(f'{j}'):
                    throttle = j / 9
                    break
            if keyboard.is_pressed('up'):
                throttle = abs(throttle)
            if keyboard.is_pressed('down'):
                throttle = -abs(throttle)
            if keyboard.is_pressed('left'):
                yaw = -0.5
            if keyboard.is_pressed('right'):
                yaw = 0.5
            if throttle != prevthrottle or yaw != prevyaw:
                motor.write(throttle, yaw)
            prevthrottle = throttle
            prevyaw = yaw
        except KeyboardInterrupt:
            break
    motor.kill()
def asm(asmstr):
    asm = []
    opcodes = {'F': forward, 'B': back, 'R': rotate, 'S': smartmove}
    asmstr = asmstr.upper()
    asmstr = asmstr.splitlines()
    asmstr = [line.strip() for line in asmstr]
    asmstr = [line for line in asmstr if not line.startswith('#')]
    for line in asmstr:
        ins = []
        line = line.split(' ')
        opcode = line[0].strip()
        if opcode in opcodes:
            ins.append(opcode)
            if len(line) > 1:
                arg = float(line[1].strip())
                ins.append(arg)
            asm.append(ins)
    for ins in asm:
        time.sleep()(1)
        if len(ins) > 1:
            opcodes[ins[0]](ins[1])
        elif len(opcodes) > 0:
            opcodes[ins[0]]()
if __name__ == '__main__':
    with open('main.asm', 'r') as file:
        asm(file.read())