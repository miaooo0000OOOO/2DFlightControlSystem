str2BGR = {'red':(0,0,255), 'green':(0,255,0), 'blue':(255,0,0), 'yellow':(0,255,255), 'black':(0,0,0), 'white':(255,255,255)}

def colorStr2BGR(color):
    if type(color) == str:
        if color[0]=='#':
            r, g, b = color[1:3], color[3:5], color[5:7]
            r, g, b = int(r,16), int(g,16), int(b,16)
            return (b,g,r)
        else:
            return str2BGR[color]
    return color
        


