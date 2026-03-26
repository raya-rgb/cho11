from controller import Robot, Motor, DistanceSensor, Camera

# ================= CONSTANTS =================
MAX_SPEED = 6.28
SPEED_RATIO = 0.5
FRONT_LIMIT = 0.02
MIN_COLOR_LEVEL = 80

# Dog color
DOG_R = 77
DOG_G = 80
DOG_B = 89
TOLERANCE = 40


# ================= SENSOR =================
def read_sensors(sensor_list):
    values = []
    for s in sensor_list:
        val = s.getValue() / 4096.0
        values.append(min(val, 1.0))
    return values


def obstacle_ahead(values):
    return (values[0] + values[7]) / 2 > FRONT_LIMIT


# ================= MOVEMENT =================
def forward(left, right):
    left.setVelocity(MAX_SPEED * SPEED_RATIO)
    right.setVelocity(MAX_SPEED * SPEED_RATIO)


def reverse(left, right, robot, step):
    left.setVelocity(-MAX_SPEED * SPEED_RATIO)
    right.setVelocity(-MAX_SPEED * SPEED_RATIO)
    delay(robot, step, 0.3)


def rotate_left(left, right, robot, step):
    left.setVelocity(-MAX_SPEED * SPEED_RATIO)
    right.setVelocity(MAX_SPEED * SPEED_RATIO)
    delay(robot, step, 0.3)


def stop(left, right):
    left.setVelocity(0)
    right.setVelocity(0)


def delay(robot, step, seconds):
    start = robot.getTime()
    while robot.getTime() < start + seconds:
        robot.step(step)


# ================= CAMERA =================
def get_center_rgb(camera):
    width = camera.getWidth()
    height = camera.getHeight()
    img = camera.getImage()

    x = width // 2
    y = height // 2

    r = camera.imageGetRed(img, width, x, y)
    g = camera.imageGetGreen(img, width, x, y)
    b = camera.imageGetBlue(img, width, x, y)

    return r, g, b


# ================= COLOR DETECTION =================
def detect_color(r, g, b):
    if r > MIN_COLOR_LEVEL and r > g * 1.2 and r > b * 1.2:
        return "Red"

    if g > MIN_COLOR_LEVEL and g > r * 1.2 and g > b * 1.2:
        return "Green"

    if b > MIN_COLOR_LEVEL and b > r * 1.2 and b > g * 1.2:
        return "Blue"

    return None


# ================= DOG DETECTION =================
def is_dog(r, g, b):
    return (abs(r - DOG_R) < TOLERANCE and
            abs(g - DOG_G) < TOLERANCE and
            abs(b - DOG_B) < TOLERANCE)


# ================= IMAGE CAPTURE =================
def capture_image(camera, image_id):
    filename = f"dog_capture_{image_id}.png"
    camera.saveImage(filename, 100)
    print("Image saved:", filename)


# ================= MAIN =================
def controller(robot):

    timestep = int(robot.getBasicTimeStep())

    # Sensors
    sensor_names = ["ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7"]
    sensors = []

    for name in sensor_names:
        s = robot.getDevice(name)
        s.enable(timestep)
        sensors.append(s)

    # Camera
    cam = robot.getDevice("camera")
    cam.enable(timestep)

    # Motors
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # Memory
    found_colors = set()
    image_counter = 0
    captured = False
    frame_counter = 0

    # ================= LOOP =================
    while robot.step(timestep) != -1:

        sensor_values = read_sensors(sensors)

        # Reduce camera processing load
        if frame_counter >= 5:

            r, g, b = get_center_rgb(cam)
            print("RGB:", r, g, b)

            # ===== DOG DETECTION =====
            if is_dog(r, g, b):
                print("DOG DETECTED")
                stop(left_motor, right_motor)

                if not captured:
                    capture_image(cam, image_counter)
                    image_counter += 1
                    captured = True

                continue  # Skip movement while detecting dog

            else:
                captured = False

            # ===== COLOR DETECTION =====
            detected = detect_color(r, g, b)

            if detected and detected not in found_colors:
                found_colors.add(detected)
                print("Detected:", detected.lower())
                print("Colors seen:", ", ".join(found_colors))

            frame_counter = 0

        else:
            frame_counter += 1

        # ===== OBSTACLE AVOIDANCE =====
        if obstacle_ahead(sensor_values):
            print("avoiding obstacle")
            reverse(left_motor, right_motor, robot, timestep)
            rotate_left(left_motor, right_motor, robot, timestep)

        else:
            print("wandering around")
            forward(left_motor, right_motor)


# ================= ENTRY =================
if __name__ == "__main__":
    robot = Robot()
    controller(robot)