#!/usr/bin/python3
import wave
import pygame
import os
from pydub.utils import mediainfo

 
def play_sound(file_path, frequency, loops = 0):
    # Play a sound file (.wav or .mp3)
    # loops = -1: repeat indefinitely
    try:
        pygame.mixer.quit()
        pygame.mixer.init(frequency=frequency)        
    except:
        print('No available audio device!')
        return False
    try:
        pygame.mixer.music.load(file_path)
    except:
        print('Load {} failed'.format(file_path))
        return False
    pygame.mixer.music.play(loops = loops)
    return True

def pause():
    try:
        pygame.mixer.music.pause()
    except:
        pass

def resume():
    try:
        pygame.mixer.music.unpause()
    except:
        pass

def stop():
    try:
        pygame.mixer.music.stop()
    except:
        pass

def play_wave(file_path, loops = 0):
    # Play a wave file
    try:
        file_wave = wave.open(file_path)
    except:
        print('Open {} failed'.format(file_path))
        return False
    freq = file_wave.getframerate()
    return play_sound(file_path, freq, loops)

def play_mp3(file_path, loops = 0):
    # Play a wave file
    try:
        freq = mediainfo(file_path)['sample_rate']             
    except:
        print('Open {} failed'.format(file_path))
        return False
    return play_sound(file_path, int(freq), loops)

def say(text):
    # Text-to-Speech
    cmd = 'pico2wave -w pico_file_wave.wav "{}"'.format(text) 
    if os.system(cmd) == 0:
        return play_wave('pico_file_wave.wav')
    else:
        print('Pico2wave failed')
        return False    
        
    
    
    

    
