import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False}) # type: ignore

import omni.timeline
from omni.isaac.core import World

class Template:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.world = World()
        self.world.scene.add_default_ground_plane()

        #callbacks blir kjørt for hvert steg i simulasjonen
        self.world.add_physics_callback('template_physics_callback', self.physics_callback)
        self.world.add_render_callback('template_render_callback', self.render_callback)
        self.world.add_timeline_callback('template_timeline_callback', self.timeline_callback)
        self.world.reset()

        self.stop_sim = False

    def physics_callback(self, dt: float):
        carb.log_info("phyics callback, dt:" + str(dt))

    def render_callback(self, data):
        carb.log_info("render callback !!")

    def timeline_callback(self, timeline_event):
        if self.world.is_stopped():
            self.stop_sim = True

    def run(self):
        self.timeline.play()

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        carb.log_warn("template simulation is closing")
        self.timeline.stop()
        simulation_app.close()

def main():
    template_app = Template()
    template_app.run()

if __name__ == "__main__":
    main()
