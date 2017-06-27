"""
This module consists of code for running a ui on the official
raspberry pi 7" touchscreen
"""

import rospy
import pygame

WIDTH = 800
HEIGHT = 480

class Touchscreen():
    def __init__(self):
        self.air_temp = None
        self.humidity = None
        self.co2 = None
        self.o2 = None
        self.water_temp = None
        self.ph = None
        self.ec = None
        self.desired_temp = 25
        self.desired_hum = 50
        self.cmd_temp = 0
        self.cmd_hum = 0
        self.white = [255,255,255]
        self.black = [0,0,0]
        self.event_queue = None

        rospy.loginfo("Initializing touchscreen")
        pygame.init()
        # pygame.mouse.set_visible(False)
        self.screen = pygame.display.set_mode((WIDTH,HEIGHT),pygame.NOFRAME)

    def refresh(self):
        try:
            self.event_queue = pygame.event.get()
            self.screen.fill(self.black)
            self.blitSensorValues()
            self.blitDesiredUI()
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

    def blitDesiredUI(self):
        def plus_temp():
            self.desired_temp += 1
        def minus_temp():
            self.desired_temp -= 1
        self.createSetPointUI(0, "Air Temperature: {}".format(self.desired_temp), plus_temp, minus_temp, self.black, self.white)

        def plus_hum():
            self.desired_hum += 1
        def minus_hum():
            self.desired_hum -= 1
        self.createSetPointUI(1, "Humidity: {}".format(self.desired_hum), plus_hum, minus_hum, self.black, self.white)

        self.createSensorCard(2, "Heater on: {}".format(bool(self.cmd_temp)), self.black, self.white, x_offset=(WIDTH/2))
        self.createSensorCard(3, "Humidifier on: {}".format(bool(self.cmd_hum)), self.black, self.white, x_offset=(WIDTH/2))

    def createSensorCard(self, pos, msg, box_color=None, text_color=None, x_offset=0):
        spacing = 6
        width = (WIDTH / 2) - spacing
        height = 50 #64
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

    def button(self,msg,x,y,w,h,inactive_color,active_color, action=None):
        clicked = False
        for event in self.event_queue:
            if event.type == pygame.MOUSEBUTTONDOWN:
                clicked = True
        mouse = pygame.mouse.get_pos()

        if x+w > mouse[0] > x and y+h > mouse[1] > y:
            pygame.draw.rect(self.screen, active_color,(x,y,w,h))
            if clicked and action is not None:
                action()
        else:
            pygame.draw.rect(self.screen, inactive_color,(x,y,w,h))

        font = pygame.font.SysFont('monospace', 25)
        textSurf, textRect = self.textObjects(msg, font, [255,255,255])
        textRect.center = ( (x+(w/2)), (y+(h/2)) )
        self.screen.blit(textSurf, textRect)


    def createSetPointUI(self, pos, msg, plus, minus, box_color=None, text_color=None, x_offset=0):
        spacing = 6
        width = (WIDTH/2) - spacing
        height = 50 #64
        box_colors = [[255,255,255], [0,0,0]]
        text_colors = [[0,0,0], [255,255,255]]
        x = (WIDTH/2) + spacing + x_offset
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
        self.button("-", x, y, height, height, [50,50,50], [200,200,200], minus)
        self.button("+", WIDTH - height - spacing, y, height, height, [50,50,50], [200,200,200], plus)
        self.screen.blit(text_surf, text_rect)

    def textObjects(self, text, font, color):
        text_surface = font.render(text, True, color)
        return text_surface, text_surface.get_rect()
