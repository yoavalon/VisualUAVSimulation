import sys
from direct.showbase.ShowBase import ShowBase
#import gltf
sys.path.insert(0, "/home/algo/code/RenderPipeline")
from rpcore import PointLight
from panda3d.core import Vec3, load_prc_file_data
from direct.task import Task
import numpy as np 
import matplotlib.pyplot as plt 
from panda3d.core import TransparencyAttrib

class Application(ShowBase):
    

    def __init__(self):
        #ShowBase.__init__()
        sys.path.insert(0, "render_pipeline")

        # Setup window size, title and so on
        load_prc_file_data("", """
            win-size 1600 900
            window-title Indoor-Robotics Vision Simulator
        """)
        

        from rpcore import RenderPipeline
        self.render_pipeline = RenderPipeline()
        from rpcore.util.movement_controller import MovementController

        #self.render_pipeline.settings['pipeline.display_debugger'] = False       
        self.render_pipeline.create(self)
        self.render_pipeline.daytime_mgr.time = "5:30" 

        #simplepbr.init()


        #gltf.patch_loader(self.loader)
        self.building = loader.loadModel("models/office16.gltf")
        
        #self.building = loader.loadModel("/home/algo/Desktop/blocks.gltf")
        #self.building = loader.loadModel("models/office9.gltf")
        #self.building = loader.loadModel('/home/algo/Downloads/test.bam')
        #self.building = loader.loadModel('/home/algo/Desktop/cubes.bam')
        #self.building = loader.loadModel('models/window4.gltf')
        #self.building.setTransparency(True)
        

        #self.building.setTransparency(True)
        self.building.setPos(0,0,0)
        self.building.setHpr(90,0,0)
        #self.building.setTransparency(True)
        #self.building.set_shader_input("detail_scale_factor", 4.0)
        render.setTransparency(TransparencyAttrib.MAlpha)
        self.building.reparentTo(render)   
        
        self.render_pipeline.prepare_scene(self.building)
        #self.building.setColor(1,1,1,0.3)

        #self.controller = MovementController(self)

        self.controller = MovementController(self)
        self.controller.set_initial_position(
            Vec3(0,0,0), Vec3(0,0,0))
        self.controller.setup()

        light = PointLight()
        # set desired properties, see below
        light.pos = (3, 20, 3)
        light.color = (1.0,0.6,0.6)
        light.energy = 10.0
        #light.ies_profile = self.render_pipeline.load_ies_profile("x_arrow.ies")

        light.casts_shadows = True
        light.shadow_map_resolution = 512
        light.near_plane = 0.2

        self.render_pipeline.add_light(light)
        self.render_pipeline.prepare_scene(self.building)

        #print(render.ls())
    
Application().run()