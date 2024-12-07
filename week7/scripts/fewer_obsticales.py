#!/usr/bin/env python
class Robot:
    def __init__(self):
        # Initial position, facing upwards (can be any initial direction)
        self.position = [0, 0]
        self.direction = 'up'  # Can be 'up', 'down', 'left', or 'right'
        
    def move_forward(self):
        """Move the robot one step forward in the current direction."""
        if self.direction == 'up':
            self.position[1] += 1
        elif self.direction == 'down':
            self.position[1] -= 1
        elif self.direction == 'left':
            self.position[0] -= 1
        elif self.direction == 'right':
            self.position[0] += 1
        print(f"Moved forward to position {self.position}")
        
    def turn_left(self):
        """Turn the robot 90 degrees to the left."""
        directions = ['up', 'left', 'down', 'right']
        self.direction = directions[(directions.index(self.direction) + 1) % 4]
        print(f"Turned left. Now facing {self.direction}")
        
    def turn_right(self):
        """Turn the robot 90 degrees to the right."""
        directions = ['up', 'right', 'down', 'left']
        self.direction = directions[(directions.index(self.direction) + 1) % 4]
        print(f"Turned right. Now facing {self.direction}")
    
    def detect_obstacle(self, direction='front'):
        """Simulate obstacle detection. In a real robot, this would use sensors."""
        # Replace with actual sensor reading logic (this is a simulation).
        import random
        return random.choice([True, False])  # Randomly returns True (obstacle) or False (no obstacle)
    
    def decide_direction(self):
        """Decide whether to turn left, right, or continue straight."""
        # Check for obstacles in front
        front_obstacle = self.detect_obstacle('front')
        if front_obstacle:
            print("Obstacle detected in front. Checking sides...")
            
            # Check for obstacles on left and right
            left_obstacle = self.detect_obstacle('left')
            right_obstacle = self.detect_obstacle('right')

            if left_obstacle and not right_obstacle:
                print("Turning right, because left is blocked.")
                self.turn_right()
            elif right_obstacle and not left_obstacle:
                print("Turning left, because right is blocked.")
                self.turn_left()
            elif not left_obstacle and not right_obstacle:
                # Both sides are clear, can choose either direction
                print("Both sides are clear. Turning left by default.")
                self.turn_left()
            else:
                # Both sides have obstacles
                print("Both sides are blocked, turning right as a fallback.")
                self.turn_right()
        else:
            print("No obstacle in front, moving forward.")
            self.move_forward()
    
    def run(self):
        """Main method to run the robot's behavior."""
        # For example, let the robot run for 10 steps (or until an exit condition is satisfied)
        for _ in range(10):
            self.decide_direction()