#This file contains code for the "Automated Tagging System" for the Tufts/Goodwill Senior Design project

import RPi.GPIO as GPIO
import time 

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the BCM pins for motor control
big_LA_pins = (17,27) 
small_LA_pins = (23,24)
stepper_pins = (5,6,13,19)
footpedal_pin = 26
button_pin = 3

# Initialize variables for debounce
button_pressed_last1 = False
button_pressed_last2 = False
debounce_delay = 1  # Adjust this value based on your requirements
checking_delay = 0.01

# Function to move the LA down
def move_down(pins):
    GPIO.output(pins[0], GPIO.HIGH)
    GPIO.output(pins[1], GPIO.LOW)

# Function to move the LA up
def move_up(pins):
    GPIO.output(pins[0], GPIO.LOW)
    GPIO.output(pins[1], GPIO.HIGH)

# Stop the LA
def stop_motor(pins):
    GPIO.output(pins[0], GPIO.LOW)
    GPIO.output(pins[1], GPIO.LOW)

# Move the stepper motor clockwise
def cw(step_number, inputdelay, pins):
    for x in range(0,step_number,4):
        try:
            GPIO.output(pins, (GPIO.HIGH,GPIO.LOW,GPIO.HIGH,GPIO.LOW))
            time.sleep(inputdelay)
            GPIO.output(pins, (GPIO.LOW,GPIO.HIGH,GPIO.HIGH,GPIO.LOW))
            time.sleep(inputdelay)
            GPIO.output(pins, (GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.HIGH))
            time.sleep(inputdelay)
            GPIO.output(pins, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
            time.sleep(inputdelay)
        except KeyboardInterrupt:
            GPIO.cleanup()

# Move the stepper motor counterclockwise
def ccw(step_number, inputdelay, pins):
    try:
        for x in range(0,step_number,4):
            GPIO.output(pins, (GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH))
            time.sleep(inputdelay)
            GPIO.output(pins, (GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH))
            time.sleep(inputdelay)
            GPIO.output(pins, (GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW))
            time.sleep(inputdelay)
            GPIO.output(pins, (GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW))
            time.sleep(inputdelay)

    except KeyboardInterrupt:
        GPIO.cleanup()

# Main program
def main():
    try:
        move_down(big_LA_pins)
        time.sleep(2.5)
        stop_motor(big_LA_pins)
        time.sleep(1)
        move_down(small_LA_pins)
        time.sleep(2)
        move_up(small_LA_pins)
        time.sleep(2)
        move_up(big_LA_pins)
        time.sleep(2.5)
        stop_motor(big_LA_pins)
        time.sleep(1.5)  
        #cw(40,0.01,stepper_pins)
        time.sleep(2)
        GPIO.cleanup()
        
    except KeyboardInterrupt:
        GPIO.cleanup()

# Activate "Automated Tagging System" with footpedal press
try:
    while True:
        # Set the GPIO mode
        GPIO.setmode(GPIO.BCM)

        # Set up the GPIO pins as OUTPUT
        GPIO.setup(big_LA_pins, GPIO.OUT)
        GPIO.setup(small_LA_pins, GPIO.OUT)
        GPIO.setup(stepper_pins, GPIO.OUT)
        GPIO.setup(footpedal_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        #Short delay to avoid false readings
        time.sleep(checking_delay)

        # Check if the footpedal is pressed
        button_state1 = GPIO.input(footpedal_pin)

        # Check if the button is pressed
        button_state2 = GPIO.input(button_pin)

        # Check for a rising edge for footpedal (button transition from not pressed to pressed)
        if not button_pressed_last1 and button_state1 == GPIO.LOW:
            print("Footpedal pressed!")
            main()
            # Add a short delay to debounce the button
            #time.sleep(debounce_delay)
            # Add your desired action or function call here

        # Check for a rising edge for button (button transition from not pressed to pressed)
        if not button_pressed_last2 and button_state2 == GPIO.LOW:
            print("Button pressed!")
            ccw(40,0.01,stepper_pins)
            # Add a short delay to debounce the button
            time.sleep(debounce_delay)
            # Add your desired action or function call here
            GPIO.cleanup()

        # Update the last button state
        button_pressed_last1 = button_state1 == GPIO.LOW

        # Update the last button state
        button_pressed_last2 = button_state2 == GPIO.LOW

except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()






