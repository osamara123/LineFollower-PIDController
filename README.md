
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
</head>
<body>

  <h1>🚗 Line Follower Robot with PID Controller & Bluetooth Speed Control</h1>

  <p>
    This embedded systems project implements an <strong>autonomous line-following robot</strong> using an Arduino UNO and a set of IR sensors. The system uses a <strong>PID controller</strong> to dynamically adjust motor speeds and ensure smooth and accurate line tracking. In addition, a <strong>Bluetooth HM-10 module</strong> allows real-time control of the robot’s maximum speed via a mobile application or serial terminal.
  </p>

  <h2>🔧 Hardware Overview</h2>
  <ul>
    <li>Arduino UNO</li>
    <li>5 IR sensors (Left, Center, Right, extras)</li>
    <li>Motor Driver (L298N or similar)</li>
    <li>DC Motors (2 or 4, depending on your configuration)</li>
    <li>Bluetooth Module (HM-10)</li>
    <li>Breadboards and jumper wires</li>
    <li>Power supply (battery pack)</li>
  </ul>

  <h3>📸 Hardware Schematic</h3>
  <p>See below for a visual layout of the physical wiring:</p>
  
  ![Image](https://github.com/user-attachments/assets/262108b1-597d-4fa1-8316-4a01a7fc005b)
 

  <h2>🧠 Software Features</h2>
  <ul>
    <li>Custom PID logic with tunable parameters (Kp, Ki, Kd)</li>
    <li>Multiple driving modes (NORMAL, EXTRA, PID, SLEW)</li>
    <li>Spiral recovery on line loss</li>
    <li>T-junction and corner detection</li>
    <li>Bluetooth interface to adjust <strong>maximum speed</strong></li>
    <li>State variables for robust sensor tracking</li>
  </ul>

  <h3>🧩 Core Functions</h3>
  <ul>
    <li><code>setup()</code> – Initializes pins, serial ports, and Bluetooth module</li>
    <li><code>loop()</code> – Main runtime logic for sensor readings and motor control</li>
    <li><code>PID Logic</code> – Runs when center sensor is active</li>
    <li><code>spiral()</code> – Called when all sensors lose the line</li>
    <li><code>setDrive()</code> – Adjusts motor speeds based on control logic</li>
  </ul>

  <h2>📱 Bluetooth Integration</h2>
  <p>
    The HM-10 Bluetooth module listens for incoming speed values and updates the robot's max allowed motor speed in real time. This allows developers or users to throttle the robot's movement without needing to reprogram the microcontroller.
  </p>

  <h2>📌 Notes</h2>
  <ul>
    <li>Ensure all sensor inputs are correctly connected to analog/digital pins as defined in the `.ino` sketch.</li>
    <li>HM-10 module default baud is typically 9600 or 115200. Adjust in `setup()` if needed.</li>
    <li>Mobile Bluetooth terminal apps (like "Serial Bluetooth Terminal" on Android) work well for testing.</li>
  </ul>

  <h2>🔒 License</h2>
  <p>This project is open-source and free to use for academic and personal purposes.</p>

</body>
</html>
