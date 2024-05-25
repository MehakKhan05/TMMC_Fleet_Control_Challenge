import TMMC_Wrapper
import rclpy
from pynput.keyboard import Listener
import numpy as np
from PIL import Image
import os
import json

# initialization
print("Initializing...")
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
		self.bot.send_cmd_vel(velocity, 0.0)

	def move_backward(self):
		if self.min_dist_back <= 0.3:
			velocity = 0.0
		else:
			velocity = min(0.8 * self.min_dist_back, 0.5)
		self.bot.send_cmd_vel(-velocity, 0.0)

	def turn_left(self):
		self.bot.send_cmd_vel(0.0, 0.5)

	def turn_right(self):
		self.bot.send_cmd_vel(0.0, -0.5)

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
			if TMMC_Wrapper.is_SIM:
				self.min_dist_front = min(self.bot.last_scan_msg.ranges[0:45] + self.bot.last_scan_msg.ranges[315:360])
			else:
				self.min_dist_front = sorted(self.bot.last_scan_msg.ranges[90:270])[2] # get third smallest
		except:
			self.min_dist_front = -1
		
		try:
			if TMMC_Wrapper.is_SIM:		
				self.min_dist_back = min(self.bot.last_scan_msg.ranges[135:225])
			else:
				self.min_dist_back = sorted(self.bot.last_scan_msg.ranges[450:630])[2] # get third smallest
		except:
			self.min_dist_back = -1
			
		if "w" in self.keys_pressed:
			self.move_forward()
		if "s" in self.keys_pressed:
			self.move_backward()
		if "a" in self.keys_pressed:
			self.turn_left()
		if "d" in self.keys_pressed:
			self.turn_right()
		
		if "i" in self.keys_pressed:
			self.keys_pressed = set()
			print("FRONT DISTANCE:", self.min_dist_front, "BACK DISTANCE:", self.min_dist_back)

		if "l" in self.keys_pressed:
			self.keys_pressed = set()
			print([round(min(10, i), 3) for i in self.bot.last_scan_msg.ranges])
		
		if "p" in self.keys_pressed:
			self.keys_pressed = set()
			try:
				img = self.bot.last_image_msg
				img_array = np.reshape(img.data, (img.height, img.width, 3))

				PIL_image = Image.fromarray(img_array.astype("uint8"), "RGB")
				PIL_image.save(os.path.join(os.path.expanduser('~'), 'robot_image.png'))
			except:
				pass

		rclpy.spin_once(self.bot, timeout_sec=0.1)

	def shutdown(self):
		self.bot.destroy_node()

ROBOT1 = Robot()

def main():
	ROBOT1.safe_keyboard_control()
	print("Listening for keyboard events.")
	while True:
		ROBOT1.eventloop()
		
try:
	main()
except KeyboardInterrupt:
	print("Keyboard interrupt receieved. Stopping...")
finally:
	ROBOT1.shutdown()
	rclpy.shutdown()
