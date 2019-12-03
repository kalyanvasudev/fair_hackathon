import Pyro4

if __name__ == '__main__':
	bot = Pyro4.Proxy("PYRONAME:remotelocorobot@192.168.0.124")
	pan_angle = bot.get_pan()
	tilt_angle = bot.get_tilt()
	bot.set_pan_tilt(pan_angle + 0.1, tilt_angle - 0.1)
