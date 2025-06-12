#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <NewPing.h>
#include "BluetoothSerial.h"
#include <ArduinoJson.h>

// Bluetooth
BluetoothSerial serialBle;

// JSON Data
JsonDocument carData;

// Ultrasonic Sensor Pinout
#define TRIG_FRONT 23 
#define ECHO_FRONT 22
#define TRIG_LEFT 32
#define ECHO_LEFT 33
#define TRIG_RIGHT 18
#define ECHO_RIGHT 19
#define MAX_DISTANCE 200

#define WALL_DISTANCE 20
#define CAR_SPEED 40
#define CAR_MOVE_DELAY 1600

NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);

int frontDist, leftDist, rightDist;

MPU6050 mpu;

// Maze Setup
#define MAZE_SIZE 5
int goalX = 2, goalY = 1;
int startX = 0, startY = 4;

enum Direction {NORTH, EAST, SOUTH, WEST};
int dx[4] = {0, 1, 0, -1};
int dy[4] = {-1, 0, 1, 0};

// Maze wall data structure
struct Cell {
  bool walls[4] = {false, false, false, false}; // N, E, S, W
  bool visited = false;
};

Cell maze[MAZE_SIZE][MAZE_SIZE];


// Manhattan Matrix
int manhattan[MAZE_SIZE][MAZE_SIZE];

void generateManhattan() {
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      manhattan[y][x] = abs(goalX - x) + abs(goalY - y);
    }
  }
}


// Sensor and wall mapping
Direction currentDir = NORTH;

Direction leftOf(Direction dir)  { return (Direction)((dir + 3) % 4); }
Direction rightOf(Direction dir) { return (Direction)((dir + 1) % 4); }
Direction backOf(Direction dir)  { return (Direction)((dir + 2) % 4); }



// Motor driver pins
int PWMA = 14, PWMB = 15;
int AIN1 = 27, AIN2 = 26;
int BIN1 = 4, BIN2 = 2;
int STBY = 25;

// PWM settings
const int freq = 15000;
const int resolution = 8; // 0-255
const int channelA = 1;
const int channelB = 2;

float yaw = 0;
unsigned long lastTime;

// Integrate gyro data to calculate yaw
void updateYaw()
{
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // in seconds
  lastTime = now;

  float gz_dps = gz / 131.0; // convert to deg/sec
  yaw += gz_dps * dt;
}

void stopMotors()
{
  carData["move"] = "S";
  String data;
  serializeJson(carData, data);
  serialBle.println(data);
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void turnRight90()
{
  carData["move"] = "R";
  String data;
  serializeJson(carData, data);
  serialBle.println(data);
  yaw = 0;
  lastTime = millis();
  // Begin slow right turn: Left motor forward, Right motor backward
  ledcWrite(PWMA, CAR_SPEED); // Low speed
  digitalWrite(AIN1, 0);
  digitalWrite(AIN2, 1);

  ledcWrite(PWMB, CAR_SPEED);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);

  while (yaw < 80)
  {
    updateYaw();
    Serial.print("Yaw: ");
    Serial.println(yaw);
    delay(2);
  }

  stopMotors();
}

void turnLeft90(){
  carData["move"] = "L";
  String data;
  serializeJson(carData, data);
  serialBle.println(data);
  yaw = 0;
  lastTime = millis();

  // Begin slow right turn: Left motor forward, Right motor backward
  ledcWrite(PWMA, CAR_SPEED); // Low speed
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);

  ledcWrite(PWMB, CAR_SPEED);
  digitalWrite(BIN1, 0);
  digitalWrite(BIN2, 1);

  while (yaw > -80)
  {
    updateYaw();
    Serial.print("Yaw: ");
    Serial.println(yaw);
    delay(2);
  }
  stopMotors();
}



void moveForward()
{
  carData["move"] = "F";
  String data;
  serializeJson(carData, data);
  serialBle.println(data);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, CAR_SPEED); // Adjust speed
  ledcWrite(PWMB, CAR_SPEED);
}


// Recursive maze scanner
void scanMaze(int x, int y, Direction dir) {
  // Step 1: Boundary and visited check
  if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) return; // Out of bounds
  if (maze[y][x].visited) return; // Already visited this cell

  maze[y][x].visited = true; // Mark this cell as visited

  // Step 2: Read ultrasonic sensor distances in cm
  frontDist = sonarFront.ping_cm();  // Distance in front of the robot
  leftDist  = sonarLeft.ping_cm();   // Distance on the left
  rightDist = sonarRight.ping_cm();  // Distance on the right

  // if any sensor reads distance close to MAX_DISTANCE, set it to MAX_DISTANCE
  if (frontDist >= MAX_DISTANCE - 10) frontDist = MAX_DISTANCE;
  if (leftDist  >= MAX_DISTANCE - 10) leftDist  = MAX_DISTANCE;
  if (rightDist >= MAX_DISTANCE - 10) rightDist = MAX_DISTANCE;

  // Step 3: Mark walls based on sensor readings
  // If wall detected in front, mark the wall in the current direction
  if (frontDist < WALL_DISTANCE) maze[y][x].walls[dir] = true;

  // If wall detected on the left, mark wall in direction to the left of current
  if (leftDist  < WALL_DISTANCE) maze[y][x].walls[leftOf(dir)] = true;

  // If wall detected on the right, mark wall in direction to the right of current
  if (rightDist < WALL_DISTANCE) maze[y][x].walls[rightOf(dir)] = true;

  // Step 4: Decide where to go next — based on unvisited neighbor with lowest Manhattan distance
  int minDist = 1000; // Start with a large number
  int nextDir = -1;   // Will store the direction to move next

  for (int i = 0; i < 4; i++) {
    int nx = x + dx[i]; // Neighbor's x
    int ny = y + dy[i]; // Neighbor's y

    // Skip out-of-bound neighbors
    if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) continue;

    // Skip if wall exists in that direction
    if (maze[y][x].walls[i]) continue;

    // Skip already visited cells
    if (maze[ny][nx].visited) continue;

    // Choose the neighbor with the minimum Manhattan distance
    if (manhattan[ny][nx] < minDist) {
      minDist = manhattan[ny][nx];
      nextDir = i; // Save this direction to go next
    }
  }

  // Step 5: If a valid direction was found, move and recurse
  if (nextDir != -1) {
    // Calculate how much to turn from current direction to desired direction
    int diff = (nextDir - dir + 4) % 4;

    // Turn right 90 degrees
    if (diff == 1) turnRight90();
    // Turn left 90 degrees
    else if (diff == 3) turnLeft90();
    // Turn around (180 degrees)
    else if (diff == 2) {
      turnRight90(); delay(300); // First 90 degrees
      turnRight90();             // Second 90 degrees
    }

    // Move forward one cell
    moveForward(); delay(CAR_MOVE_DELAY);

    // Calculate new position after move
    int nx = x + dx[nextDir];
    int ny = y + dy[nextDir];

    // Recurse into the next cell
    scanMaze(nx, ny, (Direction)nextDir);

    // Step 6: Backtrack after recursion (return path)
    // Turn around to face back
    turnRight90(); delay(300);
    turnRight90();

    // Move back to previous cell
    moveForward(); delay(CAR_MOVE_DELAY);

    // Turn around again to resume original direction
    turnRight90(); delay(300);
    turnRight90();
  }
}


void setup()
{
  serialBle.begin("MazeRobot");
  delay(100);
  Serial.begin(115200);
  Wire.begin(17, 13); // <-- updated to your custom I2C pins

  // Motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH); // Enable the motor driver

  // Initialize PWM channels
  ledcAttachChannel(PWMA, freq, resolution, channelA);
  ledcAttachChannel(PWMB, freq, resolution, channelB);

  // MPU6050
  mpu.initialize();
  if (!mpu.testConnection())
  {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }

  // Calibrate - read initial yaw
  Serial.println("Calibrating...");
  delay(1000);
  yaw = 0;
  lastTime = millis();
}

void loop()
{
  scanMaze(startX, startY, currentDir);
}

