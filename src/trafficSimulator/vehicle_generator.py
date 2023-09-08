from vehicle import Vehicle
from numpy.random import randint

class VehicleGenerator:
    def __init__(self, sim, config={}):
        self.sim = sim
        self.destinations = set()
        self.next_vehicle = 0
        self.vehicle_counter = 0

        # Set default configurations
        self.set_default_config()

        # Update configurations
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()

    def set_default_config(self):
        """Set default configuration"""
        self.vehicle_rate = 20
        self.vehicles = [
            (1, {})
        ]
        self.last_added_time = 0

    def init_properties(self):
        self.upcoming_vehicle = self.generate_vehicle()

    def generate_vehicle(self):
        """Returns a random vehicle from self.vehicles with random proportions"""
        # Se regresa el carro de la posicion vehicle_counter
        vehicle = self.vehicles[self.vehicle_counter]
        weight, config = vehicle

        self.vehicle_counter += 1
        return Vehicle(config)

    def update(self):
        """Add vehicles"""
        roadF = self.sim.roads[self.upcoming_vehicle.path[-1]]
        # Se revisa si el destinto es un estacionamiento libre
        # Y que todavia alla estacionamientos restantes
        if(roadF.tipo == 1 and self.upcoming_vehicle.path[-1] in self.destinations or self.vehicle_counter >= len(self.vehicles)):
            return
        
        if self.sim.t - self.last_added_time >= 100 / self.vehicle_rate:
            # If time elasped after last added vehicle is
            # greater than vehicle_period; generate a vehicle
            road = self.sim.roads[self.upcoming_vehicle.path[0]]
            if len(road.vehicles) == 0\
               or road.vehicles[-1].x > self.upcoming_vehicle.s0 + self.upcoming_vehicle.l:
                # If there is space for the generated vehicle; add it
                self.upcoming_vehicle.time_added = self.sim.t
                road.vehicles.append(self.upcoming_vehicle)
                self.destinations.add(self.upcoming_vehicle.path[-1])
                # Reset last_added_time and upcoming_vehicle
                self.last_added_time = self.sim.t
            self.upcoming_vehicle = self.generate_vehicle()

