import pygame, numpy, dnlv2, math, generateMesh
size=[800, 600]
scale=1.0
scr = pygame.display.set_mode(size)

camera_position = [0.0, 0.0, 15.0, 100.0] #x, y, zoom, vertical shift
cam_speed = 0.3

#time resolution
iter_step = 50

#used to create a factor of step count and fps to change dt
speed = 20.0

fps=0

#cornvert world position to screen position
def screentransform(world_pos):
    return [camera_position[2]*(world_pos[0]-camera_position[0])+size[0]/2,
            camera_position[2]*(world_pos[1]-camera_position[1])+size[1]/2]


#convert screen position to world position
def invscreentransform(screen_pos):
    return [(screen_pos[0]-size[0]/2)/camera_position[2] + camera_position[0],
            (screen_pos[1]-size[1]/2)/camera_position[2] + camera_position[1]]


playing = True
clock = pygame.time.Clock()

update_sim = True

rigid_box = []

mesh_data = [[0, 0, 0, 1, 1, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
         	 [0, 0, 1, 1, 1, 1, 0, 0],
        	 [0, 0, 1, 0, 0, 1, 0, 0],
          	 [0, 0, 1, 0, 0, 1, 0, 0]]

vertex_size=0.4
mesh_size=2.5

mesh_transform = [0, (-len(mesh_data)-1)*mesh_size, mesh_size] #x,y size

#create lists to store mesh
pointList, lineList = generateMesh.generatemesh(mesh_data, mesh_transform, [1.6, 1.6])

center_of_mass = [0, 0]

def onscreen(x, kind=0):
    if kind:
        return (onscreen([x[0], x[1]], 0) or
                onscreen([x[0]+x[2], x[1]], 0) or
                onscreen([x[0], x[1]+x[3]], 0) or
                onscreen([x[0]+x[2], x[1]+x[3]], 0))
    
    return x[0]>0 and x[0]<size[0] and x[1]>0 and x[1]<size[1]

#average of all points position
def centermesh():
    divisor = len(pointList)
    avg = [0, 0]
    for i in pointList:
        avg[0]+=i.origin[0]/divisor
        avg[1]+=i.origin[1]/divisor
    return avg


#find center of mass and rotate around it
#'a' rotates the position of the mesh
#'b' adds rotational velocity
def rotatemesh(a, b):
    avg = centermesh()
    for i in pointList:
        phase = math.atan2(i.origin[1]-avg[1], i.origin[0]-avg[0])
        magnitude = math.dist(i.origin, avg)
        i.velocity = [i.origin[0]-(math.cos(phase+b)*magnitude+avg[0])+i.velocity[0],
                      i.origin[1]-(math.sin(phase+b)*magnitude+avg[1])+i.velocity[1]]
        i.origin = [math.cos(phase+a)*magnitude+avg[0], math.sin(phase+a)*magnitude+avg[1]]


screen_pressed = 0
grabpos = [] #world position of last screen press


while playing:
    
    pos = list(pygame.mouse.get_pos())
    
    #define center of mass
    ct = centermesh()
    
    fps = clock.get_fps()
    
    sfps = str(round(fps, 2))
    
    pygame.display.set_caption("fps: "+sfps+" ; x: "+str(round(ct[0]))+" ; y: "+str(round(ct[1]))+" ; "+["paused", "unpaused"][update_sim])
    
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            playing = False
        
        if event.type==pygame.MOUSEBUTTONDOWN:
            #store world position
            if not screen_pressed:
                grabpos = invscreentransform(pos)
                screen_pressed = 1
        
        if event.type==pygame.MOUSEBUTTONUP:
            #create new box
            endpos = invscreentransform(pos)
            rigid_box.append([grabpos[0], grabpos[1], endpos[0]-grabpos[0], endpos[1]-grabpos[1]])
            screen_pressed = 0
        
        if event.type==pygame.KEYDOWN:
            
            #undo box placement
            if event.key==pygame.K_z:
                if len(rigid_box)>0: rigid_box.pop()
            
            #zoom in / out
            if event.key==pygame.K_1:
                camera_position[2]*=1.5
            if event.key==pygame.K_2:
                camera_position[2]/=1.5
            
            #increase mesh size
            if event.key==pygame.K_3:
                for i in lineList:
                    i.length*=1.5
            if event.key==pygame.K_4:
                for i in lineList:
                    i.length/=1.5
            
            #add velocty according to wasd key press
            if event.key==pygame.K_w:
                for i in pointList:
                    i.velocity[1]-=2.0
            if event.key==pygame.K_s:
                for i in pointList:
                    i.velocity[1]+=0.8
            if event.key==pygame.K_a:
                for i in pointList:
                    i.velocity[0]-=0.2
                rotatemesh(0.0, 0.1)
            if event.key==pygame.K_d:
                for i in pointList:
                    i.velocity[0]+=0.2
        
                rotatemesh(0.0, -0.1)
            
            #pause / unpause
            if event.key == pygame.K_SPACE:
                update_sim = not update_sim
    
    
    #update positions
    if update_sim and fps!=0:
        for i in range(iter_step):
            dt = speed/(fps*iter_step)
            dnlv2.update(pointList, lineList, dt, 0.3, rigid_box, vertex_size)
    
    
    #decrease difference in camera and mesh position by factor of cam_speed and fps
    #creates dragging effect
    camera_position[0]+=(ct[0]-camera_position[0])*cam_speed*fps/60.0
    camera_position[1]+=(ct[1]-camera_position[1]-camera_position[3]/camera_position[2])*cam_speed*fps/60.0
    
    
    scr.fill((255,255,255))
    
    #draw bounding box around rigid box being built
    if screen_pressed:
        vgp = screentransform(grabpos)
        pygame.draw.rect(scr, [0,0,0], [vgp[0], vgp[1], pos[0]-vgp[0], pos[1]-vgp[1]], 3)
    
    #draw filled box around existing boxes
    for i in rigid_box:
        vi = screentransform(i)
        vi2 = screentransform([i[0]+i[2], i[1]+i[3]])
        vi.append(vi2[0]-vi[0])
        vi.append(vi2[1]-vi[1])
        if onscreen(vi, 1):
            vi[3]+=min(0, size[1]-vi[1]-vi[3])
            vi[2]+=min(0, size[0]-vi[0]-vi[2])
            pygame.draw.rect(scr, [0, 0, 0], vi)
    
    #draw lines connecting verticies
    for i in lineList:
        #color based on tension of spring
        tns = max(-1, min(1, i.force*0.5))*0.5 + 0.5
        va = list(i.point_a.origin)
        va = screentransform(va)
        vb = list(i.point_b.origin)
        vb = screentransform(vb)
        pygame.draw.line(scr, [0, 0, 0], va, vb, 6)
        pygame.draw.line(scr, [255*(1-tns), 127, 255*tns], va, vb, 3)
    
    #draw floor at y=0
    vf = screentransform([0, 0])
    if vf[1]<size[1]:
        pygame.draw.rect(scr, [0, 0, 0], [0, vf[1], size[0], size[1]-vf[1]+1])
    
    #draw circle at mouse position
    pygame.draw.circle(scr, [255, 255, 255], pos, 8, 1)
    
    
    #draw circles at verticies
    for i in pointList:
        pygame.draw.circle(scr, [255, 255, 255], screentransform(i.origin), max(2, int(vertex_size*camera_position[2])), 1)
    
    #draw bounding box
    pygame.draw.rect(scr, [0,0,0], [0,0,size[0],size[1]], 3)
    
    pygame.display.update()
    clock.tick(30)