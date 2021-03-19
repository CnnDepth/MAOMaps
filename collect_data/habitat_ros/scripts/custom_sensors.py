import habitat
from typing import Any
from gym import spaces
import numpy as np

# Define the sensor and register it with habitat
# For the sensor, we will register it with a custom name
@habitat.registry.register_sensor(name="position_sensor")
class AgentPositionSensor(habitat.Sensor):
    def __init__(self, sim, config, **kwargs: Any):
        super().__init__(config=config)

        self._sim = sim

    # Defines the name of the sensor in the sensor suite dictionary
    def _get_uuid(self, *args: Any, **kwargs: Any):
        return "agent_position"

    # Defines the type of the sensor
    def _get_sensor_type(self, *args: Any, **kwargs: Any):
        return habitat.SensorTypes.POSITION

    # Defines the size and range of the observations of the sensor
    def _get_observation_space(self, *args: Any, **kwargs: Any):
        return spaces.Box(
            low=np.finfo(np.float32).min,
            high=np.finfo(np.float32).max,
            shape=(3,),
            dtype=np.float32,
        )

    # This is called whenver reset is called or an action is taken
    def get_observation(
        self, observations, *args: Any, episode, **kwargs: Any
    ):
        sensor_states = self._sim.get_agent_state().sensor_states
        return (sensor_states['rgb'].position, sensor_states['rgb'].rotation)