# Adapted from dxf2svg.py by David S. Touretzky
# Computer Science Department, Carnegie Mellon University
# Released under the GNU GPL3 license.


__author__ = 'David S. Touretzky, Stefan Hechenberger <stefan@nortd.com>'


import math
import StringIO




class DXFReader:
    """Parse very simple DXF files with lines, arcs, and lwpolyline.

    Usage:
    reader = DXFReader(0.08)
    boundarys = reader.parse(open('filename').read())
    """

    # Based on colour table here: http://sub-atomic.com/~moses/acadcolors.html
    # Color 7 is actually #ffffff but #000000 is more visible
    __DXF_COLORS = ['000000', 'ff0000', 'ffff00', '00ff00', '00ffff', '0000ff', 'ff00ff', '000000', '414141', '808080', 'ff0000', 'ffaaaa', 'bd0000', 'bd7e7e', '810000', '815656', '680000', '684545', '4f0000', '4f3535', 'ff3f00', 'ffbfaa', 'bd2e00', 'bd8d7e', '811f00', '816056', '681900', '684e45', '4f1300', '4f3b35', 'ff7f00', 'ffd4aa', 'bd5e00', 'bd9d7e', '814000', '816b56', '683400', '685645', '4f2700', '4f4235', 'ffbf00', 'ffeaaa', 'bd8d00', 'bdad7e', '816000', '817656', '684e00', '685f45', '4f3b00', '4f4935', 'ffff00', 'ffffaa', 'bdbd00', 'bdbd7e', '818100', '818156', '686800', '686845', '4f4f00', '4f4f35', 'bfff00', 'eaffaa', '8dbd00', 'adbd7e', '608100', '768156', '4e6800', '5f6845', '3b4f00', '494f35', '7fff00', 'd4ffaa', '5ebd00', '9dbd7e', '408100', '6b8156', '346800', '566845', '274f00', '424f35', '3fff00', 'bfffaa', '2ebd00', '8dbd7e', '1f8100', '608156', '196800', '4e6845', '134f00', '3b4f35', '00ff00', 'aaffaa', '00bd00', '7ebd7e', '008100', '568156', '006800', '456845', '004f00', '354f35', '00ff3f', 'aaffbf', '00bd2e', '7ebd8d', '00811f', '568160', '006819', '45684e', '004f13', '354f3b', '00ff7f', 'aaffd4', '00bd5e', '7ebd9d', '008140', '56816b', '006834', '456856', '004f27', '354f42', '00ffbf', 'aaffea', '00bd8d', '7ebdad', '008160', '568176', '00684e', '45685f', '004f3b', '354f49', '00ffff', 'aaffff', '00bdbd', '7ebdbd', '008181', '568181', '006868', '456868', '004f4f', '354f4f', '00bfff', 'aaeaff', '008dbd', '7eadbd', '006081', '567681', '004e68', '455f68', '003b4f', '35494f', '007fff', 'aad4ff', '005ebd', '7e9dbd', '004081', '566b81', '003468', '455668', '00274f', '35424f', '003fff', 'aabfff', '002ebd', '7e8dbd', '001f81', '566081', '001968', '454e68', '00134f', '353b4f', '0000ff', 'aaaaff', '0000bd', '7e7ebd', '000081', '565681', '000068', '454568', '00004f', '35354f', '3f00ff', 'bfaaff', '2e00bd', '8d7ebd', '1f0081', '605681', '190068', '4e4568', '13004f', '3b354f', '7f00ff', 'd4aaff', '5e00bd', '9d7ebd', '400081', '6b5681', '340068', '564568', '27004f', '42354f', 'bf00ff', 'eaaaff', '8d00bd', 'ad7ebd', '600081', '765681', '4e0068', '5f4568', '3b004f', '49354f', 'ff00ff', 'ffaaff', 'bd00bd', 'bd7ebd', '810081', '815681', '680068', '684568', '4f004f', '4f354f', 'ff00bf', 'ffaaea', 'bd008d', 'bd7ead', '810060', '815676', '68004e', '68455f', '4f003b', '4f3549', 'ff007f', 'ffaad4', 'bd005e', 'bd7e9d', '810040', '81566b', '680034', '684556', '4f0027', '4f3542', 'ff003f', 'ffaabf', 'bd002e', 'bd7e8d', '81001f', '815660', '680019', '68454e', '4f0013', '4f353b', '333333', '505050', '696969', '828282', 'bebebe', 'ffffff']
    __DXF_VERSIONS = {"AC1006":"R10","AC1009":"R11 and R12","AC1012":"R13","AC1014":"R14","AC1015":"AutoCAD 2000","AC1018":"AutoCAD 2004","AC1021":"AutoCAD 2007","AC1024":"AutoCAD 2010","AC1027":"AutoCAD 2013"}

    def __init__(self, tolerance):
        # tolerance settings, used in tessalation, path simplification, etc         
        self.tolerance = tolerance
        self.tolerance2 = tolerance**2

        # parsed path data, paths by color
        # {'#ff0000': [[path0, path1, ..], [path0, ..], ..]}
        # Each path is a list of vertices which is a list of two floats.        
        self.boundarys = {}

        # Assume metric by default
        self.metricflag = 1
        self.linecount = 0

        self.line = ''
        self.dxfcode = ''

        self.headers = {}
        self.blocks = {}
        self.layers = {}

    def parse(self, dxfstring):
        self.linecount = 0
        self.line = ""
        self.infile = StringIO.StringIO(dxfstring)

        while True:
            self.readtocode(0)

            if self.line == "SECTION":
                self.readtocode(2)

                if self.line == "HEADER": self.do_header()
                elif self.line == "TABLES": self.do_tables()
                elif self.line == "BLOCKS": self.do_blocks()
                elif self.line == "ENTITIES": self.do_entities()
                else: pass

            elif self.line == "EOF": break
            else: pass

        print self.layers

        self.infile.close()
        print "Done!"
        return {'boundarys': self.boundarys}

    # Read through all of the drawing variables
    def do_header(self):
        self.readonepair()

        # Read in all of the setting in the header
        while not (self.dxfcode == 0 and self.line == "ENDSEC"):
            if self.dxfcode == 9:
                key = self.line

                while True:
                    self.readonepair()
                    if self.dxfcode == 0 or self.dxfcode == 9:
                        break

                    val = (self.dxfcode, self.line)
                    if key in self.headers:
                        self.headers[key].append(val)
                    else:
                        self.headers[key] = [val]

                if len(self.headers[key]) == 1:
                    self.headers[key] = self.headers[key][0]

            else: self.complain_invalid()

        # Detect the DXF version being used
        if self.headers["$ACADVER"]:
            acadver = self.headers["$ACADVER"][1]
            if acadver in self.__DXF_VERSIONS:
                self.__log.info("DXF Version: %s" % (self.__DXF_VERSIONS[acadver]))
            else:
                self.__log.warn("Unrecognized DXF version: %s" % (acadver))

        # Read the files units setting
        if self.headers["$MEASUREMENT"]:
            self.metricflag = int(self.headers["$MEASUREMENT"][1])
            if self.metricflag == 0:
                print "Found imperial units indicator -> converting to mm."
            elif self.metricflag != 1:
                print "Invalid $MEASUREMENT value! Assuming metric units."
                self.metricflag = 1
            else:
                print "Found metric units indicator."

    # Read through all layer data
    def do_tables(self):
        self.readonepair()

        # Read in all of the setting in the header
        while True:
            if self.dxfcode == 0 and self.line == "LAYER": self.do_table_layer()
            elif self.line == "ENDSEC": break
            else: self.readonepair()

    def do_table_layer(self):
        layer_name = None
        layer_color = None
        layer_plotting = True

        while True:
            self.readonepair()
            if self.dxfcode == 0: break

            if self.dxfcode == 2: layer_name = self.line
            elif self.dxfcode == 62: layer_color = int(self.line)
            elif self.dxfcode == 290: layer_plotting = boolean(self.line)

        if layer_plotting and layer_color < 0:
            layer_color = -layer_color
            layer_plotting = False

        layer_color = "#" + self.__DXF_COLORS[layer_color]
        self.layers[layer_name] = {"color": layer_color, "plot": layer_plotting}

    # Read through all block data
    def do_blocks(self):
        while True:
            self.readtocode(0)
            if self.line == "BLOCK": self.do_block()
            elif self.line == "ENDSEC": break
            else: self.complain_invalid()

    def do_block(self):
        self.readtosection(0, "ENDBLK")

    # Read through all entity data
    def do_entities(self):
        # TODO: Read entire entity blocks before evaluating each entity
        while True:
            self.readtocode(0)
            if self.line == "LINE": self.do_line()
            elif self.line == "CIRCLE": self.do_circle()
            elif self.line == "ARC": self.do_arc()
            elif self.line == "LWPOLYLINE": self.do_lwpolyline()
            #elif self.line == "SPLINE": self.complain_spline()
            elif self.line == "ENDSEC": break
            else: print "Unknown element '" + self.line + "' on line ", self.linecount

    ################
    # Routines to read entries from the DXF file

    def readtosection(self, codeval, stringval):
        self.dxfcode = None
        while (self.dxfcode != codeval) or (self.line != stringval):
            self.readonepair()

    def readonepair(self):
        self.readoneline()
        self.dxfcode = int(self.line)
        self.readoneline()

    def readoneline(self):
        self.linecount += 1
        self.line = self.infile.readline()
        if not self.line: 
            print "Premature end of file!"
            print "Something is wrong. Sorry!"
            raise ValueError
        self.line = self.line.rstrip()

    def readtocode(self, val):
        self.dxfcode = None
        while self.dxfcode != val:
            self.readonepair()

    def readgroup(self, codeval):
        self.readtocode(codeval)
        return self.line

    ################
    # Translate each type of entity (line, circle, arc, lwpolyline)

    def do_line(self):
        x1 = float(self.readgroup(10))
        y1 = float(self.readgroup(20))
        x2 = float(self.readgroup(11))
        y2 = float(self.readgroup(21))
        if self.metricflag == 0:
            x1 = x1*25.4
            y1 = y1*25.4        
            x2 = x2*25.4
            y2 = y2*25.4        

    def do_circle(self):
        cx = float(self.readgroup(10))
        cy = float(self.readgroup(20))
        r = float(self.readgroup(40))
        if self.metricflag == 0:
            cx = cx*25.4
            cy = cy*25.4        
            r = r*25.4  
        path = []
        self.addArc(path, cx-r, cy, r, r, 0, 0, 0, cx, cy+r)
        self.addArc(path, cx, cy+r, r, r, 0, 0, 0, cx+r, cy)
        self.addArc(path, cx+r, cy, r, r, 0, 0, 0, cx, cy-r)
        self.addArc(path, cx, cy-r, r, r, 0, 0, 0, cx-r, cy)

        return path

    def do_arc(self):
        cx = float(self.readgroup(10))
        cy = float(self.readgroup(20))
        r = float(self.readgroup(40))
        if self.metricflag == 0:
            cx = cx*25.4
            cy = cy*25.4        
            r = r*25.4        
        theta1deg = float(self.readgroup(50))
        theta2deg = float(self.readgroup(51))
        thetadiff = theta2deg-theta1deg
        if thetadiff < 0 : thetadiff = thetadiff + 360
        large_arc_flag = int(thetadiff >= 180)
        sweep_flag = 1
        theta1 = theta1deg/180.0 * math.pi;
        theta2 = theta2deg/180.0 * math.pi;
        x1 = cx + r*math.cos(theta1)
        y1 = cy + r*math.sin(theta1)
        x2 = cx + r*math.cos(theta2)
        y2 = cy + r*math.sin(theta2)
        path = []
        self.addArc(path, x1, y1, r, r, 0, large_arc_flag, sweep_flag, x2, y2)
        return path

    def do_lwpolyline(self):
        numverts = int(self.readgroup(90))
        path = []
        for i in range(0,numverts):
            x = float(self.readgroup(10))
            y = float(self.readgroup(20))
            if self.metricflag == 0:
                x = x*25.4
                y = y*25.4
            path.append([x,y])

    def complain_spline(self):
        print "Encountered a SPLINE at line", self.linecount
        print "This program cannot handle splines at present."
        print "Convert the spline to an LWPOLYLINE using Save As options in SolidWorks."
        raise ValueError

    def complain_invalid(self):
        print "Invalid element '" + self.line + "' on line ", self.linecount
        print "Can't process this DXF file. Sorry!"
        raise ValueError

    def addArc(self, path, x1, y1, rx, ry, phi, large_arc, sweep, x2, y2):
        # Implemented based on the SVG implementation notes
        # plus some recursive sugar for incrementally refining the
        # arc resolution until the requested tolerance is met.
        # http://www.w3.org/TR/SVG/implnote.html#ArcImplementationNotes
        cp = math.cos(phi)
        sp = math.sin(phi)
        dx = 0.5 * (x1 - x2)
        dy = 0.5 * (y1 - y2)
        x_ = cp * dx + sp * dy
        y_ = -sp * dx + cp * dy
        r2 = ((rx*ry)**2-(rx*y_)**2-(ry*x_)**2) / ((rx*y_)**2+(ry*x_)**2)
        if r2 < 0:
            r2 = 0
        r = math.sqrt(r2)
        if large_arc == sweep:
            r = -r
        cx_ = r*rx*y_ / ry
        cy_ = -r*ry*x_ / rx
        cx = cp*cx_ - sp*cy_ + 0.5*(x1 + x2)
        cy = sp*cx_ + cp*cy_ + 0.5*(y1 + y2)
        
        def _angle(u, v):
            a = math.acos((u[0]*v[0] + u[1]*v[1]) /
                            math.sqrt(((u[0])**2 + (u[1])**2) *
                            ((v[0])**2 + (v[1])**2)))
            sgn = -1
            if u[0]*v[1] > u[1]*v[0]:
                sgn = 1
            return sgn * a
    
        psi = _angle([1,0], [(x_-cx_)/rx, (y_-cy_)/ry])
        delta = _angle([(x_-cx_)/rx, (y_-cy_)/ry], [(-x_-cx_)/rx, (-y_-cy_)/ry])
        if sweep and delta < 0:
            delta += math.pi * 2
        if not sweep and delta > 0:
            delta -= math.pi * 2
        
        def _getVertex(pct):
            theta = psi + delta * pct
            ct = math.cos(theta)
            st = math.sin(theta)
            return [cp*rx*ct-sp*ry*st+cx, sp*rx*ct+cp*ry*st+cy]        
        
        # let the recursive fun begin
        def _recursiveArc(t1, t2, c1, c5, level, tolerance2):
            def _vertexDistanceSquared(v1, v2):
                return (v2[0]-v1[0])**2 + (v2[1]-v1[1])**2
            
            def _vertexMiddle(v1, v2):
                return [ (v2[0]+v1[0])/2.0, (v2[1]+v1[1])/2.0 ]

            if level > 18:
                # protect from deep recursion cases
                # max 2**18 = 262144 segments
                return

            tRange = t2-t1
            tHalf = t1 + 0.5*tRange
            c2 = _getVertex(t1 + 0.25*tRange)
            c3 = _getVertex(tHalf)
            c4 = _getVertex(t1 + 0.75*tRange)
            if _vertexDistanceSquared(c2, _vertexMiddle(c1,c3)) > tolerance2:
                _recursiveArc(t1, tHalf, c1, c3, level+1, tolerance2)
            path.append(c3)
            if _vertexDistanceSquared(c4, _vertexMiddle(c3,c5)) > tolerance2:
                _recursiveArc(tHalf, t2, c3, c5, level+1, tolerance2)
                
        t1Init = 0.0
        t2Init = 1.0
        c1Init = _getVertex(t1Init)
        c5Init = _getVertex(t2Init)
        path.append(c1Init)
        _recursiveArc(t1Init, t2Init, c1Init, c5Init, 0, self.tolerance2)
        path.append(c5Init)





