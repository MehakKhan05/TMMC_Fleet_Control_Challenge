import TMMC_Wrapper
import rclpy
from pynput.keyboard import Listener

USING_HARDWARE = False

# initialization
TMMC_Wrapper.is_SIM = True
if not rclpy.ok():
	rclpy.init()
TMMC_Wrapper.use_hardware()

class Robot:
	def __init__(self):
		self.bot = TMMC_Wrapper.Robot()
		self.keys_pressed = set()

	def move_forward(self):
		if self.min_dist_front <= 0.3:
			velocity = 0.0
		else:
			velocity = min(0.8 * self.min_dist_front, 0.5)
		if USING_HARDWARE:
			velocity /= 2
		self.bot.send_cmd_vel(velocity, 0.0)

	def move_backward(self):
		if self.min_dist_back <= 0.3:
			velocity = 0.0
		else:
			velocity = min(0.8 * self.min_dist_back, 0.5)
		if USING_HARDWARE:
			velocity /= 2
		self.bot.send_cmd_vel(-velocity, 0.0)

	def turn_left(self):
		self.bot.send_cmd_vel(0.0, 0.25)

	def turn_right(self):
		self.bot.send_cmd_vel(0.0, -0.25)

	def stop_robot(self):
		self.bot.send_cmd_vel(0.0, 0.0)

	def safe_keyboard_control(self):
		if self.bot.keyboard_listener is None:
			def on_press(key):
				try:
					self.keys_pressed.add(key.char)
				except:
					pass
			def on_release(key):
				try:
					self.keys_pressed.remove(key.char)
				except:
					pass
			self.keyboard_listener = Listener(on_press=on_press, on_release=on_release)
			self.keyboard_listener.start()
		else:
			print("Keyboard listener already running")

	def eventloop(self):
		if len(self.keys_pressed) != 1:
			self.stop_robot()
		
		try:
			if USING_HARDWARE:
				self.min_dist_front = sorted(self.bot.last_scan_msg.ranges[90:270])[2] / 1.5 # get third smallest
			else:
				self.min_dist_front = min(self.bot.last_scan_msg.ranges[0:45] + self.bot.last_scan_msg.ranges[315:360])
		except:
			self.min_dist_front = -1
		
		try:
			if USING_HARDWARE:
				self.min_dist_back = sorted(self.bot.last_scan_msg.ranges[450:630])[2] / 1.5 # get third smallest
			else:
				self.min_dist_back = min(self.bot.last_scan_msg.ranges[135:225])
		except:
			self.min_dist_back = -1
		
		print("FRONT DISTANCE:", self.min_dist_front, "BACK DISTANCE:", self.min_dist_back)
	
		if "w" in self.keys_pressed:
			self.move_forward()
		if "s" in self.keys_pressed:
			self.move_backward()
		if "a" in self.keys_pressed:
			self.turn_left()
		if "d" in self.keys_pressed:
			self.turn_right()

		rclpy.spin_once(self.bot, timeout_sec=0.1)

ROBOT1 = Robot()

def main():
	print("running main")
	ROBOT1.safe_keyboard_control()
	print("Listening for keyboard events. Press keys to test, Ctrl C to exit")
	while True:
		ROBOT1.eventloop()
		
try:
	main()
except KeyboardInterrupt:
	print("Keyboard interrupt receieved. Stopping...")
finally:
	#when exiting program, run the kill processes
	ROBOT1.bot.stop_keyboard_control()
	ROBOT1.bot.destroy_node()
	rclpy.shutdown()
