import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

#Probably going to need to make the min/max duty cycles and the pwm freq inputs for this function
def LinearActuatorControl(pwm_frequency,duty_cycle_min,duty_cycle_max):
    
    # Set the GPIO pin for the linear actuator - this will be an output pin that connects to the relay
    actuator_pin = 18

    # Set the PWM frequency and duty cycle range - check min and max duty cycle on specs - get rid of this part once you know what the values should be and use the function inputs to define
    pwm_frequency = 50  # Hz - Controls how fast the PWM signal cycles on and off - higher PWM results in smoother and quieter operation but can limit the achievable duty clycle which can affect speed and precision of position control
    duty_cycle_min = 2  # Minimum duty cycle (0% position) - lower duty cycle results in less extension/retraction
    duty_cycle_max = 12 # Maximum duty cycle (100% position) - higher duty cycle results in more extension/retraction

    # Initialize the PWM
    GPIO.setup(actuator_pin, GPIO.OUT)
    pwm = GPIO.PWM(actuator_pin, pwm_frequency)
    pwm.start(duty_cycle_min)

    try:
        # Move the actuator to a specific position
        target_position = 7  # Adjust this value between min and max duty cycle
        pwm.ChangeDutyCycle(target_position)
        print("Moving to target position")
        time.sleep(5)  # Wait for the actuator to move
    except KeyboardInterrupt:
        pass

    # Cleanup
    pwm.stop()
    GPIO.cleanup()
    
def SolenoidControl():
    # Define the GPIO pin connected to the solenoid - this will be an output pin that connects to the relay
    solenoid_pin = 16

    # Initialize the GPIO pin as an output
    GPIO.setup(solenoid_pin, GPIO.OUT)

    try:
        # Turn on the solenoid
        GPIO.output(solenoid_pin, GPIO.HIGH)
        print("Solenoid is ON")
        time.sleep(1)  # Solenoid remains on for 1 seconds
        
        # Turn off the solenoid
        GPIO.output(relay_solenoid, GPIO.LOW)
        print("Solenoid is OFF")

    except KeyboardInterrupt:
        pass

    finally:
        # Clean up and reset the GPIO pin
        GPIO.cleanup()
 
#Will want to have servo_pin as an input so that you can control which servo is being actuated
def ServoControl(servo_pin,pwm_frequency,duty_cycle_min,duty_cycle_max)
    
    # Define the GPIO pin connected to the servo
    servo_pin = 29

    # Initialize the GPIO pin as an output
    GPIO.setup(servo_pin, GPIO.OUT)

    # Set the PWM frequency and duty cycle range - check min and max duty cycle on specs - get rid of this part once you know what the values should be and use the function inputs to define
    pwm_frequency = 50  # Hz - Controls how fast the PWM signal cycles on and off - higher PWM results in smoother and quieter operation but can limit the achievable duty clycle which can affect speed and precision of position control
    duty_cycle_min = 2  # Minimum duty cycle (0 degrees position) - lower duty cycle results in less rotation
    duty_cycle_max = 12 # Maximum duty cycle (90 degrees or 180 degrees position - whatever you want the max to be) - higher duty cycle results in more rotation

    # Initialize the PWM for servo
    GPIO.setup(servo_pin, GPIO.OUT)
    pwm1 = GPIO.PWM(servo_pin, pwm_frequency)
    pwm1.start(duty_cycle_min)
    
    try:
        # Move the servo to a specific positions
        target_position_1 = 8  # Adjust this value between min and max duty cycle
        print("Moving to target servo position")
        pwm1.ChangeDutyCycle(target_position)
        time.sleep(3)
    except KeyboardInterrupt:
        pass

    # Cleanup
    pwm.stop()
    GPIO.cleanup()

#Actuate based on button input    
try:
    #Set pin numbers for buttons
    button_pin_left = 13
    button_pin_right = 15
    
    #Set pin numbers for servos - their pins are made into outputs in the servo function
    servo_pin_1 = 29
    servo_pin_2 = 31
    
    GPIO.setup(button_pin_left, GPIO.IN)
    GPIO.setup(button_pin_right, GPIO.IN)
    
    #Actuate servo 1 to lock barb into place
    ServoControl(servo_pin_1,pwm_frequency,duty_cycle_min,duty_cycle_max)
    #Note that the linear actuator may also retract after being pushed out - we need it to hold while the barb is pushed out
    if button_pin_left and button_pin_right:
        LinearActuatorControl(pwm_frequency,duty_cycle_min,duty_cycle_max)
        time.sleep(1) #may not even need delays but putting in for now
        SolenoidControl()
        time.sleep(1)
        ServoControl(servo_pin_1,pwm_frequency,duty_cycle_min,duty_cycle_max) #unlock barbs
        time.sleep(1)
        ServoControl(servo_pin_2,pwm_frequency,duty_cycle_min,duty_cycle_max) #rotate next barb into place
        time.sleep(1)
        ServoControl(servo_pin_1,pwm_frequency,duty_cycle_min,duty_cycle_max) #lock barbs
        time.sleep(1)        
        
except KeyboardInterrupt:
    # Turn off GPIO pins
    pwm.stop()
    GPIO.cleanup()
