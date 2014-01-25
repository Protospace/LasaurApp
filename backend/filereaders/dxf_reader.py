# Adapted from dxf2svg.py by David S. Touretzky
# Computer Science Department, Carnegie Mellon University
# Released under the GNU GPL3 license.


__author__ = 'David S. Touretzky, Stefan Hechenberger <stefan@nortd.com>, Kevin Loney <kevin.loney@brainsinjars.com>'


import math
import logging
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

    __LWPOLY_CLOSED = 0x01

    __SPLINE_CLOSED = 0x01
    __SPLINE_PERIODIC = 0x02
    __SPLINE_RATIONAL = 0x04
    __SPLINE_PLANAR = 0x08
    __SPLINE_LINEAR = 0x10

    def __init__(self, tolerance):
        self.__log = logging.getLogger("dxf_reader")

        self.__entity_handlers = {
            "POINT": self.do_point,
            "LINE": self.do_line,
            "CIRCLE": self.do_circle,
            "ARC": self.do_arc,
            "LWPOLYLINE": self.do_lwpolyline,
            "SPLINE": self.do_spline,
        }

        # tolerance settings, used in tessalation, path simplification, etc         
        self.tolerance = tolerance
        self.tolerance2 = tolerance**2

        # parsed path data, paths by color
        # {'#ff0000': [[path0, path1, ..], [path0, ..], ..]}
        # Each path is a list of vertices which is a list of two floats.        
        self.boundarys = {}

        # Assume metric by default
        self.metricflag = 1

        self.splinesegs = 8

        self.line = ''
        self.dxfcode = ''
        self.linecount = 0

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

        self.__log.debug(self.layers)

        self.infile.close()
        self.__log.info('Done')

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
                self.__log.info("Found imperial units indicator, converting to mm.")
            elif self.metricflag != 1:
                self.__log.warn("Invalid $MEASUREMENT value! Assuming metric units.")
                self.metricflag = 1
            else:
                self.__log.info("Found metric units indicator.")

        # The drawing spline segments setting
        if self.headers['$SPLINESEGS']:
            self.splinesegs = int(self.headers['$SPLINESEGS'][1])

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
        self.readtocode(0)

        while True:
            if self.dxfcode != 0:
                self.readtocode(0)
            if self.dxfcode == 0 and self.line == "ENDSEC": break

            path = None
            entity_type = self.line
            entity_line = self.linecount

            layer_name = self.readgroup(8)

            if entity_type in self.__entity_handlers:
                path = self.__entity_handlers[entity_type]()
            else:
                self.__log.warn("Unknown element '%s' on line %d" % (entity_type, entity_line))
                self.readonepair()

            if path:
                layer_color = self.get_layer_color(layer_name)
                if not layer_color in self.boundarys:
                    self.boundarys[layer_color] = []
                self.boundarys[layer_color].append(path)

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
            self.__log.error("Premature end of file!")
            raise ValueError
        self.line = self.line.rstrip()

    def readtocode(self, val):
        self.dxfcode = None
        while self.dxfcode != val:
            self.readonepair()

    def readgroup(self, codeval):
        self.readtocode(codeval)
        return self.line

    def get_layer_color(self,layer_name):
        if layer_name and self.layers[layer_name]:
            return self.layers[layer_name]["color"]
        return None

    ################
    # Translate each type of entity (line, circle, arc, lwpolyline)

    def do_point(self):
        x = float(self.readgroup(10))
        y = float(self.readgroup(20))

        if self.metricflag == 0:
            x = x*25.4
            y = y*25.4

        return [[x,y]]

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

        return [[x1,y1],[x2,y2]]

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

        theta1 = math.radians(float(self.readgroup(50)))
        theta2 = math.radians(float(self.readgroup(51)))

        if self.metricflag == 0:
            cx = cx*25.4
            cy = cy*25.4
            r = r*25.4

        thetadiff = theta2 - theta1
        if thetadiff < 0:
            thetadiff = thetadiff + 2.0 * math.pi

        large_arc_flag = int(thetadiff >= math.pi)
        sweep_flag = 1

        x1 = cx + r*math.cos(theta1)
        y1 = cy + r*math.sin(theta1)
        x2 = cx + r*math.cos(theta2)
        y2 = cy + r*math.sin(theta2)

        path = []
        self.addArc(path, x1, y1, r, r, 0, large_arc_flag, sweep_flag, x2, y2)

        return path

    def do_lwpolyline(self):
        numverts = int(self.readgroup(90))
        poly_flags = int(self.readgroup(70))

        points = []
        bulge = []

        # Read vertices and any of the optional bulges
        for i in range(0, numverts):
            x = float(self.readgroup(10) if self.dxfcode != 10 else self.line)
            y = float(self.readgroup(20))

            self.readonepair()
            bulge.append(4.0 * math.atan(float(self.line)) if self.dxfcode == 42 else None)

            if self.metricflag == 0:
                x = x*25.4
                y = y*25.4

            points.append([x,y])

        # Wrap the last vertex for closed poly lines
        if poly_flags & self.__LWPOLY_CLOSED:
            points.append(points[0])
            bulge.append(None)

        path = []
        last_was_arc = False
        for i, node in enumerate(zip(bulge, points)):
            if node[0]:
                last_was_arc = True

                # Handle nodes with bulge
                # Based on conversion described at http://www.lee-mac.com/bulgeconversion.html#bulgearc
                b = node[0]
                p0 = node[1]
                p1 = points[i + 1]

                r = self._distance(p0, p1) / (2.0 * math.sin(0.5 * b))

                large_arc_flag = int(node[0] >= math.pi)
                sweep_flag = int(r > 0.0)

                self.addArc(path, p0[0], p0[1], r, r, 0, large_arc_flag, sweep_flag, p1[0], p1[1])
            elif last_was_arc:
                # Avoid replication of arc end points
                last_was_arc = False
            else:
                path.append(node[1])

        return path

    def do_spline(self):
        flags = int(self.readgroup(70))
        degree = int(self.readgroup(71))
        num_knots = int(self.readgroup(72))
        num_controls = int(self.readgroup(73))

        # Read the knot vector
        knots = []
        for i in range(0, num_knots):
            knots.append(float(self.readgroup(40)))

        # Read control points and any of the optional weights
        weights = []
        controls = []
        for i in range(0, num_controls):
            x = float(self.readgroup(10) if self.dxfcode != 10 else self.line)
            y = float(self.readgroup(20))
            z = float(self.readgroup(30))

            self.readonepair()
            weights.append(float(self.line) if self.dxfcode == 41 else 1.0)

            if self.metricflag == 0:
                x = x*25.4
                y = y*25.4
                z = z*25.4

            controls.append([x, y, z])

        path = []
        self.addSpline(path, degree, controls, knots, weights, bool(flags & self.__SPLINE_PERIODIC))
        return path

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

    def addSpline(self, path, degree, controls, x, weights, periodic):
        # Implementation described in "An Introduction to NURBS" by David F. Rogers
        # Google books: http://books.google.ca/books?id=MaW4XiScJ7cC&lpg=PA1&pg=PP1#v=onepage&q&f=false
        # Example source: http://www.nar-associates.com/nurbs/c_code.html

        npts = len(controls)

        order = degree + 1
        nplusc = npts + order

        def _rationalBasis(t):
            nplusc = npts + order;

            # calculate the first order nonrational basis functions n[i]
            temp = []
            for i in range(1, nplusc):
                temp.append(1 if ((t >= x[i - 1]) and (t < x[i])) else 0)

            # calculate the higher order nonrational basis functions
            for k in range(2, order):
                for i in range(0, nplusc - k):
                    # if the lower order basis function is zero skip the calculation
                    d = temp[i] * ((t-x[i]))/(x[i+k-1]-x[i]) if temp[i] != 0 else 0

                    # if the lower order basis function is zero skip the calculation
                    e = temp[i+1] * ((x[i+k]-t))/(x[i+k]-x[i+1]) if temp[i + 1] != 0 else 0

                    temp[i] = d + e

            if t == float(x[nplusc - 1]):
                temp[npts] = 1

            # calculate sum for denominator of rational basis functions
            total = 0
            for i in range(0, npts):
                total += temp[i + 1] * weights[i]

            r = []
            # normalize rational basis
            for i in range(0, npts):
                r.append((temp[i + 1] * weights[i])/(total) if total != 0 else 0)

            return r

        def _evalPoint(t):
            p = [0.0, 0.0, 0.0]

            # Get the basis function at t
            nbasis = _rationalBasis(t)

            # Perform matrix mutiplication between basis and control points
            for j in range(0, 3):
                for i in range(0, npts):
                    temp = nbasis[i] * controls[i][j]
                    p[j] += temp

            # Only interested in the x and y coordinates
            return p[0:2]

        def _recursiveSpline(level, t1, p1, t2, p2):
            def _distPointSegment(p, u, v):
                # Calculate distance between a point and a segment using vector projection

                l2 = self._distance2(u, v)
                if l2 == 0.0:
                    # Zero length segment case
                    return self._distance(p, u)

                t = self._vectorDot(self._vectorSub(p, u), self._vectorSub(v, u)) / l2
                if t < 0.0:
                    # Before the start of the segment
                    return self._distance(p, u)
                if t > 1.0:
                    # After the end of the segment
                    return self._distance(p, v)

                # Distance between point and it's projection onto the segment
                return self._distance(p, [u + t * (v - u) for u,v in zip(u, v)])

            if level > 18:
                # protect from deep recursion cases
                # max 2**18 = 262144 segments
                return

            t0 = 0.5 * (t1 + t2)
            p0 = _evalPoint(t0)

            if _distPointSegment(p0, p1, p2) > self.tolerance2:
                _recursiveSpline(level + 1, t1, p1, t0, p0)

                path.append(p0[0:2])

                _recursiveSpline(level + 1, t0, p0, t2, p2)

        # Evaluate the NURBS curve
        t_prev = 0.0
        pt_prev = _evalPoint(t_prev)
        path.append(pt_prev)

        for i in range(1, npts):
            t_curr = x[nplusc - 1] * float(i) / (npts - 1)
            pt_curr = _evalPoint(t_curr)

            _recursiveSpline(0, t_prev, pt_prev, t_curr, pt_curr)

            path.append(pt_curr)

            t_prev = t_curr
            pt_prev = pt_curr

    def _vectorDot(self, u, v):
        return sum([a*b for a, b in zip(u, v)])

    def _vectorSub(self, u, v):
        return [u - v for u,v in zip(u, v)]

    def _distance2(self, u, v):
        t = self._vectorSub(u, v)
        return self._vectorDot(t, t)

    def _distance(self, u, v):
        return math.sqrt(self._distance2(u, v))