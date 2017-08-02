import queue
import _thread
import time
import pygame

from tmp_navigation_practice import run_navigation

finish_flag = 0

def do_navigation(name, delay):
	time.sleep(delay)
	print(name)
		
	run_navigation()
	finish_flag = 1

def play_music(name, delay):
	count = 0
	time.sleep(delay)
	print(name)
	file='JJlin.mp3' #文件名是完整路径名
	#file = file.encode('utf-8')

	pygame.mixer.init() #初始化音频
	track = pygame.mixer.music.load(file)#载入音乐文件
	pygame.mixer.music.play(loops = 10, start = 10.5)#开始播放
	pygame.mixer.music.set_volume(0.2)
	if finish_flag == 1:
		print('stop playing')
		pygame.mixer.music.stop()#开始播放


def combine_2():
	try:
		_thread.start_new_thread( do_navigation, ("navigation", 0, ))
		_thread.start_new_thread( play_music,("music", 1, ))
	except:
		print ("Error: 无法启动线程")


	while 1:
		pass


if __name__ == '__main__':
	combine_2()
