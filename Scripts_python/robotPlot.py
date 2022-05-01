from mathLib import Vector
import matplotlib.pyplot as plt

#Robot constants (in mm)
distance_captor_max_range = 20
wheel_dist = 53
wheel_diameter = 13
robot_diameter = 70
wheel_radius = wheel_diameter/2
robot_radius = robot_diameter/2

class Capteur_dist:
    def __init__(self,position: Vector, direction: Vector,angle_rel):
        self.position = position
        self.direction = direction
        self.angle_rel = angle_rel
        self.direction.r = distance_captor_max_range
        self.arrow = plt.arrow(self.position.x, self.position.y, self.direction.x, self.direction.y, length_includes_head = True, width = 0.01)
        self.captions = []

    def update(self,position,direction):
        self.position = position
        self.direction = direction
        self.direction.r = distance_captor_max_range
        self.arrow.set_data(x=self.position.x, y=self.position.y, dx=self.direction.x, dy=self.direction.y)
    
    def add_caption(self,dist):
        cap_r = self.direction
        cap_r.r = dist
        cap = self.position + cap_r
        plotcap = plt.plot(cap.x, cap.y, "ro", ms = 1)
        self.captions.append(plotcap)
    
    def remove_captions(self):
        for caption in self.captions:
            point = caption.pop(0)
            point.remove()
        self.captions = []



class Robot:

    def __init__(self, fig, ax, position = Vector(r=1,theta=-90), direction = 40,capteur_angles = [0,45,-45,90,-90]):
        self.fig = fig
        self.ax = ax
        self.command = None
        self.position = position
        self.direction = Vector(r=1,theta=direction)

        self.capteurs = []
        for capteur_angle in capteur_angles:
            position_c_abs,position_c_rel = self.compute_capteur_position(capteur_angle)
            capteur = Capteur_dist(position_c_abs,position_c_rel,capteur_angle)
            self.capteurs.append(capteur)

        self.circle = plt.Circle(self.position.xy, robot_radius,fill=True, ec= 'black',fc= 'white')
        self.ax.add_artist(self.circle)

        x_arrow, y_arrow, dx_arrow, dy_arrow = self.compute_arrow_position()
        self.arrow = plt.arrow(x_arrow, y_arrow, dx_arrow, dy_arrow,length_includes_head = True, width = 0.1*robot_radius)

        x_right_data, y_right_data = self.compute_wheel_position('right')
        self.right_wheel = plt.Line2D(x_right_data, y_right_data)

        x_left_data, y_left_data = self.compute_wheel_position('left')
        self.left_wheel = plt.Line2D(x_left_data, y_left_data)
        self.ax.add_line(self.right_wheel)
        self.ax.add_line(self.left_wheel)

        
        self.coord = ax.text(10 * robot_diameter * 1.5, 0, f'Legend:\nX = {"%.3f" % self.position.x} Y= {"%.3f" % self.position.y} \nTheta = {"%.3f" % (self.direction.theta % 360)}', style='italic',
            bbox={'facecolor':'red', 'alpha':0.5, 'pad':10})

    def compute_capteur_position(self,capteur_angle):
        position_c_rel = Vector(r = robot_radius, theta = capteur_angle + self.direction.theta)
        position_c_abs = self.position + position_c_rel
        return position_c_abs, position_c_rel

    def compute_arrow_position(self):
        self.direction.r = 1
        x_arrow = self.position.x-robot_radius*self.direction.x
        y_arrow = self.position.y-robot_radius*self.direction.y
        dx_arrow =  robot_diameter*(self.direction.x)
        dy_arrow =  robot_diameter*(self.direction.y)

        return x_arrow, y_arrow, dx_arrow, dy_arrow

    def compute_wheel_position(self,side):
        if side == 'right':
            alpha = -90
        elif side == 'left':
            alpha = 90
        else:
            return None

        center_wheel = self.position + Vector(r=wheel_dist/2,theta=self.direction.theta + alpha)
        min_wheel = center_wheel + Vector(r=wheel_radius,theta=self.direction.theta)
        max_wheel = center_wheel + Vector(r=wheel_radius,theta=self.direction.theta + 180)
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

        self.coord.set_text(f'Legend:\nX = {"%.3f" % self.position.x} Y= {"%.3f" % self.position.y} \nTheta = {"%.3f" % (self.direction.theta % 360)}')

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
    
    def remove_captor_captions(self):
        for captor in self.capteurs:
            captor.remove_captions()

    def reset(self):
        self.position = Vector(r=1,theta=-90)
        self.direction = Vector(r=1,theta=40)
        self.remove_captor_captions()
        self.update_capteurs()
        self.update_robot_plot()
