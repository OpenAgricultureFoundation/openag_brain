"""
This module consists of code for running a ui on the official
raspberry pi 7" touchscreen
"""

import rospy
import pygame

class Touchscreen():
    def __init__(self):
        self.air_temp = None
        self.humidity = None
        self.co2 = None
        self.o2 = None
        self.water_temp = None
        self.ph = None
        self.ec = None
        self.white = [255,255,255]
        self.black = [0,0,0]

        rospy.loginfo("Initializing touchscreen")
        pygame.init()
        # pygame.mouse.set_visible(False)
        self.screen = pygame.display.set_mode((800,480),pygame.NOFRAME)

    def refresh(self):
        try:
            self.screen.fill(self.black)
            self.blitSensorValues()
            pygame.display.update()
        except Exception as e:
            rospy.logwarn("Refresh crashed. Error: {}".format(e))


    def blitSensorValues(self):
        if self.air_temp is not None:
            self.createSensorCard(0, 'Air Temp: {0:.1f} C'.format(self.air_temp), self.black, self.white)
        else:
            self.createSensorCard(0, 'Air Temp: {}'.format(self.air_temp), self.black, self.white)

        if self.humidity is not None:
            self.createSensorCard(1, 'Humidity: {0:.1f} %'.format(self.humidity), self.black, self.white)
        else:
            self.createSensorCard(1, 'Humidity: {}'.format(self.humidity), self.black, self.white)

        if self.co2 is not None:
            self.createSensorCard(2, 'CO2: {0:.0f} ppm'.format(self.co2), self.black, self.white)
        else:
            self.createSensorCard(2, 'CO2: {}'.format(self.co2), self.black, self.white)

        if self.o2 is not None:
            self.createSensorCard(3, 'O2: {0:.1f} %'.format(self.o2), self.black, self.white)
        else:
            self.createSensorCard(3, 'O2: {}'.format(self.o2), self.black, self.white)

        if self.water_temp is not None:
            self.createSensorCard(4, 'Water Temp: {0:.1f} C'.format(self.water_temp), self.black, self.white)
        else:
            self.createSensorCard(4, 'Water Temp: {}'.format(self.water_temp), self.black, self.white)

        if self.ph is not None:
            self.createSensorCard(5, 'pH: {0:.1f}'.format(self.ph), self.black, self.white)
        else:
            self.createSensorCard(5, 'pH: {}'.format(self.ph), self.black, self.white)

        if self.ec is not None:
            self.createSensorCard(6, 'EC: {0:.1f} ms/cm'.format(self.ec), self.black, self.white)
        else:
            self.createSensorCard(6, 'EC: {}'.format(self.ec), self.black, self.white)

    def createSensorCard(self, pos, msg, box_color=None, text_color=None, x_offset=0):
        width = 316
        height = 50 #64
        spacing = 6
        box_colors = [[255,255,255], [0,0,0]]
        text_colors = [[0,0,0], [255,255,255]]
        x = x_offset
        y_offset = 50
        y = y_offset + (height + spacing) * pos
        font_style = 'monospace' # monospace or 'freesans.ttf'
        font_size = 25 #23 or 30

        if box_color is None:
            box_color = box_colors[pos%2]
        if text_color is None:
            text_color = text_colors[pos%2]

        pygame.draw.rect(self.screen, box_color, (x,y,width,height))
        font = pygame.font.SysFont(font_style, font_size)
        text_surf, text_rect = self.textObjects(msg, font, text_color)
        text_rect.center = ( (x+(width/2)), (y+(height/2)) )
        self.screen.blit(text_surf, text_rect)

    def textObjects(self, text, font, color):
        text_surface = font.render(text, True, color)
        return text_surface, text_surface.get_rect()
