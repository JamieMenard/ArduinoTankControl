import displayio
from adafruit_display_shapes import Rectangle, Circle
from adafruit_ble import PeripheralService, Characteristic

# Button configuration (adjust pin based on your setup)
BUTTON_PIN = Board.BUTTON

# BLE service and characteristic UUIDs (replace with custom values if desired)
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHARACTERISTIC_UUID = "fedcba09-1234-5678-1234-56789abcdef0"

# Display dimensions (replace with your PyPortal's display size)
DISPLAY_WIDTH = 240
DISPLAY_HEIGHT = 320

# Colors
RED = (255, 0, 0)
WHITE = (255, 255, 255)

# Create the display group
display = displayio.Group()

# Define the button shape (adjust size and position as needed)
button_width = 150
button_height = 100
button_x = (DISPLAY_WIDTH - button_width) // 2
button_y = (DISPLAY_HEIGHT - button_height) // 2
button_outline = Rectangle(button_x, button_y, button_width, button_height, fill=None, outline=RED, stroke_width=3)
button_inner = Rectangle(button_x + 5, button_y + 5, button_width - 10, button_height - 10, fill=RED)
x_center = button_x + button_width // 2
x_top = button_y + button_height // 3
x_bottom = button_y + (2 * button_height) // 3
x_left = button_x + button_width // 3
x_right = button_x + (2 * button_width) // 3
x_line1 = Line(x_left, x_top, x_right, x_bottom, color=WHITE, thickness=3)
x_line2 = Line(x_right, x_top, x_left, x_bottom, color=WHITE, thickness=3)

# Add shapes to the display group
display.append(button_outline)
display.append(button_inner)
display.append(x_line1)
display.append(x_line2)

# Define a helper function to send data over BLE
def send_button_press(pressed):
    characteristic.send(bytes([pressed]))  # Send True/False for button state

# Set up BLE advertisement and service
service = PeripheralService(SERVICE_UUID)
characteristic = service.Characteristic(CHARACTERISTIC_UUID, properties=Characteristic.WRITE)
characteristic.write_callback = send_button_press  # Set callback for writing to characteristic

# Show the display
show(display)

# Main loop
last_button_state = False
while True:
    button_state = digitalio.DigitalInOut(BUTTON_PIN)
    button_state.switch_to_input(pull=digitalio.Pull.UP)  # Set button with pull-up resistor

    # Check for button press and send data over BLE only on state change
    if button_state.value != last_button_state:
        send_button_press(button_state.value)
        last_button_state = button_state.value
