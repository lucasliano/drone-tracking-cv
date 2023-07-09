import matplotlib.pyplot as plt



class Vector():
    def __init__(self, name, x0, y0, z0, units):
        self.units = units
        self.name = name
        self.x = x0
        self.y = y0
        self.z = z0

    def __str__(self) -> str:
        return f'{self.name}: [{self.x},{self.y},{self.z}] {self.units}'
        


class DroneModel():
    def __init__(self, plot = True):
        self.pos = Vector('pos', 0, 0, 0, units='m')
        self.vel = Vector('vel', 0, 0, 0, units='m/s')
        self.acc = Vector('acc', 0, 0, 0, units='m/s²')
        self.heading = 0

        self.pos_setpoint = Vector('pos', 0, 0, 0, units='m')
        self.vel_setpoint = Vector('vel', 0, 0, 0, units='m/s')
        self.acc_setpoint = Vector('acc', 0, 0, 0, units='m/s²')
        self.yaw_setpoint = 0

        if plot:
            _, self.pos_axs = self.create_plots()   # El primero es la posición y el segundo el setpoint.

    def create_plots(self):
        pos_fig, pos_axs = plt.subplots(4)
        pos_fig.suptitle('Drone Position')
        plt.show(block=False)

        # Single configuration
        pos_axs[0].set_ylabel('X [m]')
        pos_axs[1].set_ylabel('Y [m]')
        pos_axs[2].set_ylabel('Z [m]')
        pos_axs[3].set_ylabel('Yaw [rad]')

        
        for ax in pos_axs:
            # General configuration  
            ax.set(
                xlabel='Time [s]',
                title='',
            )

            # Grid
            ax.grid(which='major', alpha=0.5)

        return pos_fig, pos_axs

    def calculate_offset(self, center_offset, target_depth):
        x_offset    = center_offset['x']
        y_offset    = center_offset['y']
        depth       = target_depth

        # FIXME:    Acá tenemos que hacer los cálculos pertinentes para poder convertir 
        #           de pixeles y profundidad a offset de yaw, z e x. 

        # NOTE:     Tener en cuenta que moverse en +x es ir hacia adelante.

        yaw = 1
        z   = 1
        x   = 1

        return yaw, z, x

    def update_setpoints(self, target_center_offset: dict, target_depth: float):

        yaw_offset, z_offset, x_offset = self.calculate_offset(target_center_offset, target_depth)

        self.pos_setpoint.x = self.pos.x    + x_offset
        self.pos_setpoint.z = self.pos.z    + z_offset
        self.yaw_setpoint   = self.heading  + yaw_offset    # NOTE: Hay que ver como convertir yaw en heading

    
