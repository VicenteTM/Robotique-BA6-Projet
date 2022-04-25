from mathLib import Vector
import matplotlib.pyplot as plt

#Robot constants
distance_captor_max_range = 0.4
wheel_dist = 1
wheel_diameter = 1
robot_diameter = 5
wheel_radius = wheel_diameter/2
robot_radius = robot_diameter/2

class Capteur_dist:
    def __init__(self,position,direction,angle_rel):
        self.position = position
        self.direction = direction
        self.angle_rel = angle_rel
        self.direction.r = distance_captor_max_range
        self.arrow = plt.arrow(self.position.x, self.position.y, self.direction.x, self.direction.y, length_includes_head = True, width = 0.01)

    def update(self,position,direction):
        self.position = position
        self.direction = direction
        self.direction.r = distance_captor_max_range
        self.arrow.set_data(x=self.position.x, y=self.position.y, dx=self.direction.x, dy=self.direction.y)


class Robot:

    def __init__(self, fig, ax, position = Vector(r=1,theta=-90), direction = 40,capteur_angles = [0,45,-45,90,-90]):
        self.fig = fig
        self.ax = ax
        self.command = 5
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