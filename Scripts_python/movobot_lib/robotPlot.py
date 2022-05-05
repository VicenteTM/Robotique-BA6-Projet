import math
import matplotlib.pyplot as plt
from movobot_lib.mathLib import Vector
import movobot_lib.goodcalibration as goodcalibration

#Robot constants (in mm)
DISTANCE_CAPTOR_MAX_RANGE = 20
WHEEL_DIST = 53 
WHEEL_DIAMETER = 41
WHEEL_RADIUS = WHEEL_DIAMETER/2 
WHEEL_PERIMETER = WHEEL_DIAMETER * math.pi
ROBOT_DIAMETER = 70 
ROBOT_RADIUS = ROBOT_DIAMETER/2 
STEPS_PER_TURN = 1000

class Capteur_dist:
    def __init__(self,position: Vector, direction: Vector,angle_rel, ax):
        self.position = position
        self.direction = direction
        self.angle_rel = angle_rel
        self.ax = ax
        self.threshold = 40

        self.direction.r = DISTANCE_CAPTOR_MAX_RANGE
        self.arrow = plt.arrow(self.position.x, self.position.y, self.direction.x, self.direction.y, length_includes_head = True, width = 0.01)
        self.captions = []

    def update(self,position,direction):
        self.position = position
        self.direction = direction
        self.direction.r = DISTANCE_CAPTOR_MAX_RANGE
        self.arrow.set_data(x=self.position.x, y=self.position.y, dx=self.direction.x, dy=self.direction.y)
    
    def add_caption(self,intensity, calibrated):
        cap_r = self.direction
        captor_distance = convert_intensity_to_mm(intensity ,calibrated)
        if captor_distance < self.threshold:
            cap_r.r = captor_distance
            cap = self.position + cap_r
            plotcap = self.ax.plot(cap.x, cap.y, "ro", ms = 1)
            self.captions.append(plotcap)
    
    def remove_captions(self):
        for caption in self.captions:
            point = caption.pop(0)
            point.remove()
        self.captions = []



class Robot:

    def __init__(self, fig, ax, position = Vector(r=1,theta=-90), direction = 0,capteur_angles = [-20,20,-45,45,-90,90]):
        self.fig = fig
        self.ax = ax
        self.command = None
        self.position = position
        self.direction = Vector(r=1,theta=direction)
        self.calibrated = False

        self.capteurs = []
        for capteur_angle in capteur_angles:
            position_c_abs,position_c_rel = self.compute_capteur_position(capteur_angle)
            capteur = Capteur_dist(position_c_abs,position_c_rel,capteur_angle,self.ax)
            self.capteurs.append(capteur)

        self.circle = plt.Circle(self.position.xy, ROBOT_RADIUS,fill=True, ec= 'black',fc= 'white')
        self.ax.add_artist(self.circle)

        x_arrow, y_arrow, dx_arrow, dy_arrow = self.compute_arrow_position()
        self.arrow = plt.arrow(x_arrow, y_arrow, dx_arrow, dy_arrow,length_includes_head = True, width = 0.1*ROBOT_RADIUS)

        x_right_data, y_right_data = self.compute_wheel_position('right')
        self.right_wheel = plt.Line2D(x_right_data, y_right_data)

        x_left_data, y_left_data = self.compute_wheel_position('left')
        self.left_wheel = plt.Line2D(x_left_data, y_left_data)
        self.ax.add_line(self.right_wheel)
        self.ax.add_line(self.left_wheel)

        
        self.coord = ax.text(5 * ROBOT_DIAMETER * 1.5 + self.position.x, self.position.y, f'Legend:\nX = {"%.0f" % self.position.x} Y= {"%.0f" % self.position.y} \nTheta = {"%.0f" % (self.direction.theta % 360)}', style='italic',
            bbox={'facecolor':'red', 'alpha':0.5, 'pad':10})

    def compute_capteur_position(self,capteur_angle):
        position_c_rel = Vector(r = ROBOT_RADIUS, theta = capteur_angle + self.direction.theta)
        position_c_abs = self.position + position_c_rel
        return position_c_abs, position_c_rel

    def compute_arrow_position(self):
        self.direction.r = 1
        x_arrow = self.position.x-ROBOT_RADIUS*self.direction.x
        y_arrow = self.position.y-ROBOT_RADIUS*self.direction.y
        dx_arrow =  ROBOT_DIAMETER*(self.direction.x)
        dy_arrow =  ROBOT_DIAMETER*(self.direction.y)

        return x_arrow, y_arrow, dx_arrow, dy_arrow

    def compute_wheel_position(self,side):
        if side == 'right':
            alpha = -90
        elif side == 'left':
            alpha = 90
        else:
            return None

        center_wheel = self.position + Vector(r=WHEEL_DIST/2,theta=self.direction.theta + alpha)
        min_wheel = center_wheel + Vector(r=WHEEL_RADIUS,theta=self.direction.theta)
        max_wheel = center_wheel + Vector(r=WHEEL_RADIUS,theta=self.direction.theta + 180)
        x_data = [min_wheel.x, max_wheel.x]
        y_data = [min_wheel.y, max_wheel.y]
        return x_data, y_data

    def update_capteurs(self):
        for capteur in self.capteurs:
            position_c_rel,position_c_abs = self.compute_capteur_position(capteur.angle_rel)
            capteur.update(position_c_rel,position_c_abs)

    def update_robot_plot(self):
        self.circle.set(center=self.position.xy)
        x_arrow, y_arrow, dx_arrow, dy_arrow = self.compute_arrow_position()
        self.arrow.set_data(x = x_arrow, y = y_arrow,dx = dx_arrow, dy = dy_arrow)

        x_right_data, y_right_data = self.compute_wheel_position('right')
        x_left_data, y_left_data = self.compute_wheel_position('left')
        self.right_wheel.set(xdata = x_right_data, ydata = y_right_data)
        self.left_wheel.set(xdata = x_left_data, ydata = y_left_data)

        self.coord.set_position((5 * ROBOT_DIAMETER * 1.5 + self.position.x, self.position.y))
        self.coord.set_text(f'Legend:\nX = {"%.0f" % self.position.x} Y= {"%.0f" % self.position.y} \nTheta = {"%.0f" % (self.direction.theta % 360)}')
        
        sizefromrobot = 5 * ROBOT_DIAMETER

        self.ax.set_xlim([-sizefromrobot + self.position.getx(), sizefromrobot + self.position.getx()])
        self.ax.set_ylim([-sizefromrobot + self.position.gety(), sizefromrobot + self.position.gety()])

    def move(self, d_r =None, d_theta=None):
        if d_r is not None:
            position_f_rel = Vector(r = d_r,theta=self.direction.theta)
            self.position = self.position + position_f_rel
            self.update_capteurs()
            self.update_robot_plot()
            self.fig.canvas.draw()
        elif d_theta is not None:
            self.direction.theta += d_theta
            self.update_capteurs()
            self.update_robot_plot()
            self.fig.canvas.draw()
    
    def update_robot_from_reception(self, data):
        self.position.setxy(data[0],data[1])
        self.direction.settheta(data[2] * 90)
        self.add_captor_caption(data[3:len(self.capteurs)+3])
        self.update_capteurs()
        self.update_robot_plot()
        self.fig.canvas.draw()


    def add_captor_caption(self,captions):
        for captor, caption in zip(self.capteurs,captions):
            captor.add_caption(caption, self.calibrated)
    
    def remove_captor_captions(self):
        for captor in self.capteurs:
            captor.remove_captions()

    def reset(self):
        self.position = Vector(r=1,theta=-90)
        self.direction = Vector(r=1,theta=40)
        self.remove_captor_captions()
        self.update_capteurs()
        self.update_robot_plot()



def convert_intensity_to_mm(intensity, calibrated):
    if not calibrated:
        intensity = closest_value(goodcalibration.Intensity, intensity)
        index = goodcalibration.Intensity.index(intensity)
        distance = goodcalibration.Distance[index]
        return distance
    else:
        try:
            import calibration
            intensity = closest_value(calibration.Intensity, intensity)
            index = calibration.Intensity.index(intensity)
            distance = calibration.Distance[index]
        except FileNotFoundError:
            intensity = closest_value(goodcalibration.Intensity, intensity)
            index = goodcalibration.Intensity.index(intensity)
            distance = goodcalibration.Distance[index]
        return distance



def convert_steps_to_mm(number_step):
    distance = (number_step * WHEEL_PERIMETER * 2) / (STEPS_PER_TURN * 10)
    return distance
    

def closest_value(input_list, input_value):
    difference = lambda input_list : abs(input_list - input_value)
    res = min(input_list, key=difference)

    return res
