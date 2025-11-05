import gpiod
import time
from periphery import PWM
import atexit


'''
There are usually 5 groups (also called banks) of GPIOs on the Rockchip platform: GPIO0 ~ GPIO4 (varies from chip to chip), each group has 32 GPIOs. Each group of GPIOs is divided into 4 groups, A/B/C/D, with 8 GPIOs in each group (0~7, also called bank_idx). rockchip.h)). So the GPIOs can be named from GPIO0_A0 to GPIO4_D7. Taking GPIO3_C5 as an example, we can deduce that its bank is 3 and its bank_idx is 21, i.e. GPIO3_C5 is the 21st GPIO in group 3.

The GPIO bank of Rockchip platform corresponds to gpiochip in libgpiod, and bank_idx corresponds to gpio line. Take GPIO3_C5 as an example, it is in libgpiod, gpiochip is 3 and line is 21.
'''
class TB6612MotorDriver:
    def __init__(self, \
                 ain2_line=11,      # GPIO3_B4
                 ain1_line=12,      # GPIO3_B3
                 stby_line=19,      # GPIO3_C3
                 bin1_line= 4,      # GPIO3_A4
                 bin2_line= 7,      # GPIO3_A7
                 laser_line = 6,    # GPIO3_A6
                 pwm_a_channel=12,  # PWM12_M1
                 pwm_b_channel=14,  # PWM14_M1
                 frequency=1e2,
                 speed=0.2):
        # self.chip4 = gpiod.Chip("4", gpiod.Chip.OPEN_BY_NUMBER)
        self.chip3 = gpiod.Chip("3", gpiod.Chip.OPEN_BY_NUMBER)
        # self.chip1 = gpiod.Chip("1", gpiod.Chip.OPEN_BY_NUMBER)
        
        # Request lines
        
        self.ain1 = self.chip3.get_line(ain1_line)
        self.ain1.request(consumer='motor_ain1', type=gpiod.LINE_REQ_DIR_OUT)
        self.ain1.set_value(0)
        
        self.ain2 = self.chip3.get_line(ain2_line)
        self.ain2.request(consumer='motor_ain2', type=gpiod.LINE_REQ_DIR_OUT)
        self.ain2.set_value(0)

        self.stby = self.chip3.get_line(stby_line)
        self.stby.request(consumer='motor_stby', type=gpiod.LINE_REQ_DIR_OUT)
        self.stby.set_value(0)

        self.bin1 = self.chip3.get_line(bin1_line)
        self.bin1.request(consumer='motor_bin1', type=gpiod.LINE_REQ_DIR_OUT)
        self.bin1.set_value(0)
        
        self.bin2 = self.chip3.get_line(bin2_line)
        self.bin2.request(consumer='motor_bin2', type=gpiod.LINE_REQ_DIR_OUT)
        self.bin2.set_value(0)

        self.laser = self.chip3.get_line(laser_line)
        self.laser.request(consumer='laser', type=gpiod.LINE_REQ_DIR_OUT)
        self.laser.set_value(0)

        
        # PWM setup
        self.pwm_a = PWM(pwm_a_channel, 0)
        self.pwm_b = PWM(pwm_b_channel, 0)
        self.pwm_a.frequency = frequency
        self.pwm_b.frequency = frequency
        self.pwm_a.polarity = "normal"
        self.pwm_b.polarity = "normal"
        self.pwm_a.duty_cycle = speed
        self.pwm_b.duty_cycle = speed
        self.pwm_a.enable()
        self.pwm_b.enable()
        
        self.duty_cycle = speed
        
        # Register cleanup
        atexit.register(self.cleanup)
    
    def set_speed(self, speed):
        """Set PWM duty for both motors, speed 0 to period_ns"""
        if speed == 0:
            self.pwm_a.duty_cycle = 0
            self.pwm_b.duty_cycle = 0
        else:
            self.pwm_a.duty_cycle = speed
            self.pwm_b.duty_cycle = speed
        self.duty_cycle = speed
    
    def forward(self):
        self.ain1.set_value(1)
        self.ain2.set_value(0)
        self.bin1.set_value(1)
        self.bin2.set_value(0)
        self.set_speed(self.duty_cycle)
        self.stby.set_value(1)
    
    def backward(self):
        self.ain1.set_value(0)
        self.ain2.set_value(1)
        self.bin1.set_value(0)
        self.bin2.set_value(1)
        self.set_speed(self.duty_cycle)
        self.stby.set_value(1)
    
    def left(self):
        # Left motor backward, right forward
        self.ain1.set_value(0)
        self.ain2.set_value(1)
        self.bin1.set_value(1)
        self.bin2.set_value(0)
        self.set_speed(self.duty_cycle)
        self.stby.set_value(1)
    
    def right(self):
        # Left forward, right backward
        self.ain1.set_value(1)
        self.ain2.set_value(0)
        self.bin1.set_value(0)
        self.bin2.set_value(1)
        self.set_speed(self.duty_cycle)
        self.stby.set_value(1)
    
    def stop(self):
        self.ain1.set_value(0)
        self.ain2.set_value(0)
        self.bin1.set_value(0)
        self.bin2.set_value(0)
        # self.set_speed(0)
        self.stby.set_value(0)

    def set_laser(self, state: int):
        if state == 0 or state == 1:
            self.laser.set_value(state)

    def cleanup(self):
        self.stop()
        self.set_laser(0)
        self.pwm_a.disable()
        self.pwm_b.disable()
        # self.chip1.close()
        self.chip3.close()

# Example usage
if __name__ == '__main__':
    motor = TB6612MotorDriver()
    
    try:
        motor.set_laser(1)

        motor.forward()
        time.sleep(2)
        motor.backward()
        time.sleep(2)
        motor.left()
        time.sleep(2)
        motor.right()
        time.sleep(2)

        motor.stop()
        motor.set_laser(0)
    except KeyboardInterrupt:
        motor.cleanup()