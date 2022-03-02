import sys
from direct.showbase.ShowBase import ShowBase
import time 

#accesss to render pipeline 
sys.path.insert(0, "/home/algo/code/RenderPipeline") 
from rpcore import PointLight
from direct.task import Task

#for depth buffer
from panda3d.core import FrameBufferProperties, WindowProperties
from panda3d.core import GraphicsPipe, GraphicsOutput
from panda3d.core import Texture
from panda3d.core import PNMImage
from panda3d.core import TransparencyAttrib

#vision utils
import numpy as np 
import matplotlib.pyplot as plt 
import cv2
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import Vec3, load_prc_file_data

#base.movie 

class Application(ShowBase):

    def __init__(self):
        sys.path.insert(0, "render_pipeline")

        # Setup window size, title and so on
        load_prc_file_data("", """
            win-size 800 450 
            window-title Indoor-Robotics Vision Simulator
        """)

        from rpcore import RenderPipeline
        self.render_pipeline = RenderPipeline()
      
        #self.showbase = ShowBase
        self.render_pipeline.create(self)
        self.render_pipeline.daytime_mgr.time = "05:30" 
        
        
        #obtain display region 
        self.dr = self.camNode.getDisplayRegion(0)

        self.building = loader.loadModel("models/office16.gltf")
        self.building.setPos(0,0,0)
        self.building.setHpr(90,0,0)
        self.building.reparentTo(render)   
        base.disableMouse()    

        #added for transparency
        render.setTransparency(TransparencyAttrib.MAlpha)
        self.building.reparentTo(render)           
        self.render_pipeline.prepare_scene(self.building)


        #depth code 
        winprops = WindowProperties.size(self.win.getXSize(), self.win.getYSize())
        fbprops = FrameBufferProperties()
        fbprops.setDepthBits(1)
        self.depthBuffer = self.graphicsEngine.makeOutput(
            self.pipe, "depth buffer", -2,
            fbprops, winprops,
            GraphicsPipe.BFRefuseWindow,
            self.win.getGsg(), self.win)
        self.depthTex = Texture()
        self.depthTex.setFormat(Texture.FDepthComponent)
        self.depthBuffer.addRenderTexture(self.depthTex,
            GraphicsOutput.RTMCopyRam, GraphicsOutput.RTPDepth)
        lens = self.cam.node().getLens()
        self.depthCam = self.makeCamera(self.depthBuffer,
            lens=lens,
            scene=render)
        self.depthCam.reparentTo(self.cam)

        #tasks
        self.taskMgr.add(self.movementTask, "MovementTask")
        #self.taskMgr.add(self.visionTask, "VisionTask")     #Extract image
        #self.taskMgr.add(self.sensorTask, "SensorTask")     #Extract depth 

        s = np.linspace(0,100,300)
        self.pathY = 0.1*np.sin(s)
        self.step = 0
        
        textObject = OnscreenText(text='AI Vision Simulator 0.2', pos=(0.8, -0.95), scale=0.06, fg=(1,1,1, 0.4))
        textObject = OnscreenText(text='Indoor Robotics', pos=(-0.95, -0.95), scale=0.06, fg=(235/255,140/255,52/255, 0.6))

        #new motion using movement controller
        from rpcore.util.movement_controller import MovementController
        self.controller = MovementController(self)
        self.controller.set_initial_position(
            Vec3(30,0,20), Vec3(0,0,0))
        self.controller.setup()

        base.accept("l", self.tour)
        #base.movie('./data/', duration=407, fps=24, format='png')

        '''
        light = PointLight()
        light.pos = (4,10,3)
        light.color = (1.0,0.7,0.7)
        light.energy = 50.0
        light.casts_shadows = True
        light.shadow_map_resolution = 512
        light.near_plane = 0.2

        self.render_pipeline.add_light(light)
        self.render_pipeline.prepare_scene(self.building)
        '''

        #helper 
        self.points = []

    def movementTask(self, task):

        try :

            if self.task_mgr.hasTaskNamed('RP_CameraMotionPath') == False : 
                self.tour()
                print('<movementTask> started new tour ')

                time.sleep(1)
                poi = np.array(self.points)
                plt.scatter(poi[:,0], poi[:,1], s = 2, marker = '+')
                plt.gca().set_aspect('equal', adjustable='box')
                plt.grid()
                plt.savefig('positions.png', dpi=300)
                

        except Exception as E : 
            print('<movement> ', E)

        return Task.cont

    def visionTask(self, task) : 
        """
        Extract vision 
        """
        
        tex = self.dr.getScreenshot()
        
        try :

            pos = self.controller.showbase.camera.getPos()
            angle = self.controller.showbase.camera.getHpr()

            #TODO don't save images from loading screen
            p = PNMImage()
            base.win.getScreenshot(p)
            #p.write(f'./data/0_{pos[0]}_{pos[1]}_{angle[0]}.png')
            ts = str(round(time.time()*1000))
            p.write(f'./data/image{ts}.png')

            #base.movie('./data/test.avi')

            #this one not
            #tex = base.win.getScreenshot() # this gives you a texture object
            #buf = tex.getRamImage().getData() # 800*600*4 = 1920000
            #cv2.imwrite(f'./data/0_{pos[0]}_{pos[1]}_{angle[0]}.png', buf)
            #p.stdin.write(buf) #

        except Exception as E : 
            print('<vision> ', 'FAILED', E)  

        return Task.cont    

    def sensorTask(self, task) : 
        """
        Extract sensor data
        """        
       
        try : 

            #get depth image
            data = self.depthTex.getRamImage()
            depth_image = np.frombuffer(data, np.float32)
            depth_image.shape = (self.depthTex.getYSize(), self.depthTex.getXSize(), self.depthTex.getNumComponents())
            depth_image = np.flipud(depth_image)
            
            #print('<sensor> ', 'produced depth')

            ts = str(round(time.time()*1000))
            plt.imshow(depth_image)         #saving takes long -> for that reason direct 
            plt.savefig(f'./data/depth{ts}.png')
            
            #ts = str(round(time.time()*1000))
            #cv2.imwrite(f'./data/depth{ts}.png', depth_image*255)

            #get position 
            #pos = base.camera.getPos()
        
            #pos = self.controller.showbase.camera.getPos()
            #self.points.append(pos)

        except Exception as E : 
            print('<sensor> ', 'FAILED', E)  

        return Task.cont                             

    def tour(self):
        """ Camera flythrough """
        
        
        #TODO read PATH from config file 


        template1 = [
            [[2.5,25.5,4], [-180,0,0]],
            [[2.5,25.5,1.5], [-180,0,0]],
            [[2.5,23,1.5], [-180,-5,0]],
            [[2.5,21,1.5], [-180,-2,0]],
            [[2.5,19,1.5], [-180,0,0]],
            [[2.5,16,1.5], [-180,-1,0]],
            [[2.5,12,1.5], [-180,-1,0]],
            [[2.5,8,1.5], [-180,-1,0]],
            [[2.5,6,1.5], [-180,5,0]],
            [[2.5,6,1.5], [-140,0,0]],
            [[2.5,6,1.5], [-100,0,0]],
            [[2.5,6,1.5], [-70,0,0]],
            [[2.5,6,2], [-40,0,0]],
            [[2.5,6,2], [0,0,0]],
            [[2.5,8,2], [0,-5,0]],
            [[2.5,12,2], [0,-3,0]],
            [[2.5,16,2], [0,-3,0]],
            [[2.5,20,2], [0,-3,0]],
            [[2.5,23,2], [0,6,0]],
            [[2.5,23,2], [-40,6,0]],
            [[2.5,23,2], [-80,6,0]],
            [[2.5,23,2], [-130,6,0]],
            [[2.5,23,4], [-180,6,0]],
        ]

        template2 = [
            [[2.5,25.5,4], [-180,0,0]],
            [[2.5,25.5,1.5], [-180,0,0]],
            [[2.5,23,1.5], [-180,-5,0]],
            [[2.5,21,1.5], [-180,-2,0]],
            [[2.5,19,1.5], [-180,0,0]],
            [[2.5,16,1.5], [-180,-1,0]],
            [[2.5,16,2], [-160,0,0]],
            [[2.5,16,2], [-180,0,0]],
            [[2.5,13,2], [-180,-4,0]],
            [[2.5,10,2], [-180,0,0]],
            [[2.5,6,2], [-180,0,0]],
            [[2.5,4,1.5], [-150,0,0]],
            [[2.5,4,1.5], [-120,0,0]],
            [[2.5,4,1.5], [-90,0,0]],
            [[4,4,1.5], [-90,0,0]],
            [[6,4,2], [-90,0,0]],
            [[8,4,2], [-90,0,0]],
            [[8,4,2], [-45,0,0]],
            [[8,4,2], [0,0,0]],
            [[8,7,2], [0,-5,0]],
            [[8,10,2], [0,-2,0]],
            [[8,10,2], [45,0,0]],
            [[8,10,2], [90,0,0]],
            [[4,10,2], [90,-5,0]],
            [[2.5,10,2], [135,0,0]],
            [[2.5,11,2], [180,0,0]],
        ]

        template3 = [[[2.5,2,4], [0,0,0]],
        [[2.5,2,2], [0,-5,0]],
        [[2.5,4,2], [0,0,0]],
        [[2.5,6,2], [0,0,0]],
        [[2.5,9,2], [0,0,0]],
        [[2.5,12,2], [0,1,0]],
        [[2.5,15,2], [0,-1,0]],
        [[2.5,18,1.5], [0,0,0]],
        [[2.5,22,1.5], [0,4,0]],
        [[2.5,22,1.0], [-50,0,0]],
        [[2.5,22,1.0], [-100,0,0]],
        [[2.5,22,1.0], [-160,0,0]],
        [[2.5,22,1.5], [-180,0,0]],
        [[2.5,19,2], [-180,-5,0]],
        [[2.5,17,2], [-180,3,0]],
        [[2.5,17,2], [-150,0,0]],
        [[2.5,17,2], [-180,0,0]],
        [[2.5,15,2], [-180,0,0]],
        [[2.5,12,2], [-180,0,0]],
        [[2.5,9,2], [-180,0,0]],
        [[2.5,6,2], [-180,0,0]],
        ]


        dec = np.random.rand()
        if dec > 0.6 : 
            template = template1
        elif dec >0.3 :
            template = template2
        else : 
            template = template3  

        template = template1

        r = np.random.rand(len(template),6)-0.5
        va = [0.5,0.5,0.5, 2,2,2]  #0.5 factor for variance 

        path = []
        for i in range(len(template)) : 
            path.append((Vec3(va[0]*r[i][0]+template[i][0][0],va[1]*r[i][1]+template[i][0][1],va[2]*r[i][2]+template[i][0][2]), Vec3(va[3]*r[i][3]+template[i][1][0],va[4]*r[i][4]+template[i][1][1],va[5]*r[i][0]+template[i][1][2])))

        mopath = tuple(path)
        
        self.controller.play_motion_path(mopath, 2.0)


Application().run()
