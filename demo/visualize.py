#!/usr/bin/env python3
import json
from tracemalloc import start

from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math
import os
from ast import literal_eval as make_tuple

from scipy.misc import face
plt.rcParams["font.family"] = "Times New Roman"
scale=20
#Colors = ['orange']#, 'blue', 'green']
PURPLE=(102/255.,0,255/255.)
BLUE=(0.0,193.0/255.0,232.0/255.)
GREEN=(0,176.0/255.0,80.0/255.0)
ORANGE=(255./255.,192.0/255.0,0)
PINK=(255.0/255.0,102.0/255.0,153.0/255.0)
RED=(1.0,0,0)
#Colors=[PURPLE,PURPLE,BLUE,BLUE,GREEN,GREEN,ORANGE,ORANGE,PINK,PINK,RED,RED]
#Colors=[BLUE for i in range(8)]
#Colors=[BLUE,RED]
Colors=[ORANGE]


def dist(v1,v2):
    return abs(v1[0]-v2[0])+abs(v1[1]-v2[1])


def load_instance(input_txt):
	with open(input_txt, "r") as file_content:
		lines = file_content.readlines()
		starts = list()
		goals = list()
		map_line=lines[0].split('=')
		map_name=(map_line[1].split('.'))[0]
		map_sizeX=int((map_name.split('x'))[0])
		map_sizeY=int((map_name.split('x'))[1])
		for line in lines[6:]:
			x1, y1, x2, y2 = line.split(',')
			starts.append((int(x1), int(y1)))
			goals.append((int(x2), int(y2)))
		#graph=nx.grid_graph(dim=[map_size,map_size])
		# pg.write_graph(graph,'./tmp/tmp.map')
		# pg.write_instance(graph,starts,goals,'./tmp/tmp.map','./tmp/tmp.instance')
	return map_sizeX,map_sizeY,starts,goals
	

		



	

class Animation:
    def __init__(self,system_info, sol_info):
        self.map_sizeX = system_info["xmax"]
        self.map_sizeY= system_info["ymax"]
        #self.schedule = schedule
        self.num_retrieval=system_info["num_retrieval"]
        # self.tasks=sol_info["tasks"]
        self.paths=sol_info["paths"]
        self.stations=system_info["IOports"]
        self.slots=system_info["ParkingSlots"]
        
        # self.packages=dict()
        # self.num_types=warehouse_info["num_types"]
        # self.bin_type=dict()
        self.num_robots=len(self.paths)
        print("num of robots=",self.num_robots)
        # self.obstacles=[]
        # self.package_plot=dict()
        
        # for i in range(self.num_types):
        #     for bin in warehouse_info["assignment"][str(i)]:
        #         #print(bin)
        #         self.obstacles.append((bin[0],bin[1]))
        #         self.bin_type[(bin[0],bin[1])]=i
        
        # for x in range(2,self.map_sizeX,3):
        #     for y in range(2,self.map_sizeY,3):
        #         self.obstacles.append((y,x))
        # self.stations=warehouse_info["station_positions"]
        # self.num_stations=warehouse_info["num_stations"]
        #aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]
        aspect=0.5
        sizex=16
        self.fig = plt.figure(frameon=False, figsize=(sizex , sizex))
        self.ax = self.fig.add_subplot(111, aspect=0.5)
        self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        # create boundary patch
        xmin = -0.5
        ymin = -0.5
        xmax = self.map_sizeX - 0.5
        ymax =self.map_sizeY - 0.5

        # self.ax.relim()
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)

        print("starting!")
        
        # self.type_color=dict()
        # for i in range(warehouse_info["num_types"]):
        #     self.type_color[i]=np.random.random(size=3)
        
    
        self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red'))
        
        
        self.colors=Colors
        for o in self.slots:
            x, y = o[0], o[1]
            # type=self.bin_type[(x,y)]
            self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor=BLUE, edgecolor='black',alpha=0.2))
        self.T = 0
        # draw goals first
        for s in self.stations:
            self.patches.append(Rectangle((s[0]-0.25,s[1]-0.25),0.5,0.5,facecolor=GREEN, edgecolor='black', alpha=0.15))
	
        #for  i in range(0,len(paths)):
        #  goali=self.paths[i][-1]
        #  self.patches.append(Rectangle((goali[0] - 0.25, goali[1] - 0.25), 0.5, 0.5, facecolor=self.colors[i%len(self.colors)], edgecolor='black', alpha=0.15))
        # #self.patches.append(Rectangle((d["goal"][0] - 0.25, d["goal"][1] - 0.25), 0.5, 0.5, facecolor=self.colors[i%len(self.colors)], edgecolor='black', alpha=0.15))
        for i in range(0,self.num_robots):
            ci=None
            if i<self.num_retrieval:
                ci=RED
            else:
                ci=ORANGE
                
            #name = d["name"]
            name=str(i)
            starti=self.paths[i][0]
            # self.agents[i] = Circle((starti[0], starti[1]), 0.5, facecolor=ci, edgecolor='black',linewidth=3)
         
            self.agents[i] = Rectangle((starti[0]-0.5, starti[1]-0.5), 1,1, facecolor=ci, edgecolor='black',linewidth=3)
            self.agents[i].original_face_color = self.colors[i%len(self.colors)]
            self.patches.append(self.agents[i])
            self.T = max(self.T, len(self.paths[i])-1)
            # self.agent_names[name] = self.ax.text(starti[0], starti[1], name,fontsize=10)
            self.agent_names[i] = self.ax.text(starti[0], starti[1], str(i),fontsize=10,color="white")
            self.agent_names[i].set_horizontalalignment('center')
            self.agent_names[i].set_verticalalignment('center')
            self.artists.append(self.agent_names[i])
        print("agents created!")

        # self.ax.set_axis_off()
        # self.fig.axes[0].set_visible(False)
        # self.fig.axes.get_yaxis().set_visible(False)

        # self.fig.tight_layout()
        self.init_func()
        self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                                    init_func=self.init_func,
                                    frames=int(self.T+1) * scale,
                                    interval=scale,
                                    blit=True,repeat=False)
        #self.show()
        # FFwriter = animation.FFMpegWriter(fps=10)
        # self.anim.save('one_car_path.mp4',fps=20)
	
	

            




    def save(self, file_name, speed):
        self.anim.save(
        file_name,
        "ffmpeg",
        fps=10 * speed,
        dpi=800),
        # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    def show(self):
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        # for i,p in self.package_plot.items():
          
        #     self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        # self.next_goal_id=[0 for i in range(self.num_robots)] 
        return self.patches + self.artists#+list(self.package_plot.values())

    def animate_func(self, i):
        for k in range(0,len(self.paths)):
            #agent = schedule["schedule"][agent_name]
            path=self.paths[k]
            #pos = self.getState(i / scale, agent)
            pos = self.getState(i / scale, path)
            p = (pos[0], pos[1])
            #self.agents[k].center = p
            self.agents[k].set_xy((p[0]-0.5,p[1]-0.5))
            self.agent_names[k].set_position(p)
           
        return self.patches + self.artists
    
    
    def animate_func_lifelong(self,i):
        for k in range(self.num_robots):
            path=self.paths[k]
            pos=self.getState(i/scale,path)
            p=(pos[0],pos[1])
            # print("current position=",p)
            self.agents[k].center=p
            if k in self.package_plot:
                
                self.package_plot[k].set_xy((p[0]-0.2,p[1]-0.2))
            goalk,type=self.get_current_task(k)
            if dist(p,goalk)<1e-2:
                self.next_goal_id[k]=self.next_goal_id[k]+1
                if type!=-1:
                    self.package_plot[k].remove()
                    self.package_plot.pop(k)
                if type==-1:
                    goalk,type=self.get_current_task(k)
                    assert(type!=-1)
                    # print("type=",type)
                    self.package_plot[k]=Rectangle((p[0]-0.2,p[1]-0.2),0.4,0.4,facecolor=self.type_color[type], edgecolor='black', alpha=1)
                    self.ax.add_patch(self.package_plot[k])
                    
        # print(self.package_plot)
        return self.patches + self.artists+list(self.package_plot.values())
    
    def get_current_task(self,agent_id):
        taski=self.tasks[agent_id][self.next_goal_id[agent_id]]       
        return (taski[0],taski[1]),taski[2]
        
    def getState(self, t, d):
        idx = 0
        while idx < len(d) and idx < t:
            idx += 1
        if idx == 0:
            return np.array([float(d[0][0]), float(d[0][1])])
        elif idx < len(d):
            posLast = np.array([float(d[idx-1][0]), float(d[idx-1][1])])
            posNext = np.array([float(d[idx][0]), float(d[idx][1])])
        else:
            return np.array([float(d[-1][0]), float(d[-1][1])])

        t=t-(idx-1)
        pos = (posNext - posLast) * t + posLast
        return pos





if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map")
    parser.add_argument("schedule", help="schedule for agents")
    parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
    parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
    args = parser.parse_args()

    f1=open(args.map)
    system_info=json.load(f1)
    
    f2=open(args.schedule)
    sol_info=json.load(f2)

    
    #map_sizeX,map_sizeY,starts,goals=load_instance(args.map)

    animation = Animation(system_info,sol_info)

    if args.video:
        animation.save(args.video, args.speed)
    else:
        animation.show()
