import dnlv2, numpy, math
"""
1. use imported image to create verticies around corners, ignoring previously written corners
2. use nearest neighbor algorithm to find 4 closest points
3. connect them in a box with diagonals, and solve for overlap
"""



def generatemesh(mData, mTransform, sprg):

    mSize = [len(mData[0])-1, len(mData)-1]
    
    pointList = []
    lineList = []

    #creates lines to fill boxes, only if the line is unique
    
    def makesqr(vts):
        for y in range(4):
            for x in range(y+1,4):
                unq = 1
                for i in lineList:
                        if ((vts[x]==i.point_a) and (vts[y]==i.point_b)) or ((vts[y]==i.point_a) and (vts[x]==i.point_b)): unq=0
                if unq:
                    lineList.append(dnlv2.line(vts[y],
                                               vts[x],
                                               math.dist(vts[y].origin, vts[x].origin),
                                               0.0,
                                               sprg[0],
                                               sprg[1]))


    #fill points list with corners while ignoring dupes
    
    for y in range(mSize[1]+1):
        for x in range(mSize[0]+1):
            if mData[y][x]:
                pointList.append(dnlv2.point([mTransform[0]+x*mTransform[2], mTransform[1]+y*mTransform[2]],[0, 0], [0, 0], [0, 0],0))
                l = [x==0 or (mData[y][x-1]==0),
                     y==0 or (mData[y-1][x]==0),
                     y==0 or x==mSize[0] or (mData[y-1][x+1]==0)]
                h = [0, 0]
                if (x==mSize[0] or not(mData[y][x+1])) and l[1] and l[2]:
                    pointList.append(dnlv2.point([mTransform[0]+(x+1)*mTransform[2], mTransform[1]+y*mTransform[2]],[0, 0], [0, 0], [0, 0],0))
                    
                if (y==mSize[1] or not(mData[y+1][x])) and l[0]:
                    pointList.append(dnlv2.point([mTransform[0]+x*mTransform[2], mTransform[1]+(y+1)*mTransform[2]],[0, 0], [0, 0], [0, 0],0))

                if (x==mSize[0] or y==mSize[1]) or not(mData[y+1][x+1]):
                    pointList.append(dnlv2.point([mTransform[0]+(x+1)*mTransform[2], mTransform[1]+(y+1)*mTransform[2]],[0, 0], [0, 0], [0, 0],0))


    #create 4 corners using nearest neighbor algorithm, and send to makesqr
    
    for y in range(mSize[1]+1):
        for x in range(mSize[0]+1):
            if mData[y][x]:
                pos = [mTransform[0]+x*mTransform[2] + mTransform[2]//2, mTransform[1]+y*mTransform[2] + mTransform[2]//2]
                
                #find 4 closest
                ndex = [0, 0, 0, 0]
                ndst = [2000, 2000, 2000, 2000]

                for i in range(len(pointList)):
                    dst = abs(pointList[i].origin[0]-pos[0])+abs(pointList[i].origin[1]-pos[1])
                    for j in range(4):
                        if ndst[j]>dst:
                            #ndex[j]=pointList[i]
                            ndex = (ndex[0:j]+[pointList[i]]+ndex[j:4])[0:4]
                            ndst = (ndst[0:j]+[dst]+ndst[j:4])[0:4]
                            break
                makesqr(ndex)
    return [pointList, lineList]
