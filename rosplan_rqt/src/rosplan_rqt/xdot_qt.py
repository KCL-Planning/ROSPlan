# NOTE: Important, this file can be safely removed once one of this PR gets accepted:
# https://github.com/ros-visualization/executive_smach_visualization/pull/22
# https://github.com/jbohren/xdot/pull/18
# to import DotWidget simply do:
# from smach_viewer.xdot.xdot_qt import DotWidget
# or
# from xdot.xdot_qt import DotWidget

#!/usr/bin/env python
#
# Copyright 2008 Jose Fonseca
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# QT Version adapted by Alex Bravo

'''Visualize dot graphs via the xdot format.'''

__author__ = "Jose Fonseca"

__version__ = "0.4"

import os
import sys
import subprocess
import math
import colorsys
import time
import re

from python_qt_binding import QT_BINDING_VERSION

#from PySide.QtCore import *
#from PySide.QtGui import *
if QT_BINDING_VERSION.startswith('4'):
    from PyQt4 import *
    from PyQt4.QtCore import *
    from PyQt4.QtGui import *
elif QT_BINDING_VERSION.startswith('5'):
    from PyQt5 import *
    from PyQt5.QtCore import *
    from PyQt5.QtGui import *
    from python_qt_binding.QtWidgets import QWidget, QMainWindow
else:
    raise ValueError('Unsupported Qt version, supported versions: PyQt4, PyQt5')

#from python_qt_binding import  *
#from python_qt_binding.QtCore import  *
#from python_qt_binding.QtGui import  *

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
#-- Drawing Classes --#

class Pen:
    """Store pen attributes."""

    def __init__(self):
        # set default attributes
        self.color = (0.0, 0.0, 0.0, 1.0)
        self.fillcolor = (0.0, 0.0, 0.0, 1.0)
        self.linewidth = 1.0
        self.fontsize = 14.0
        self.fontname = "Times-Roman"
        self.dash = Qt.SolidLine

    def copy(self):
        """Create a copy of this pen."""
        pen = Pen()
        pen.__dict__ = self.__dict__.copy()
        return pen

    def highlighted(self):
        pen = self.copy()
        pen.color = (1, 0, 0, 1)
        pen.fillcolor = (1, .8, .8, 1)
        return pen

class Shape:
    """Abstract base class for all the drawing shapes."""

    def __init__(self):
        pass

    def draw(self, cr, highlight=False):
        """Draw this shape with the given cairo context"""
        raise NotImplementedError

    def select_pen(self, highlight):
        if highlight:
            if not hasattr(self, 'highlight_pen'):
                self.highlight_pen = self.pen.highlighted()
            return self.highlight_pen
        else:
            return self.pen

class TextShape(Shape):
    """Used to draw a text shape with a QPainter"""

    LEFT, CENTER, RIGHT = -1, 0, 1

    def __init__(self, pen, x, y, j, w, t):
        Shape.__init__(self)
        self.pen = pen.copy()
        self.x = x
        self.y = y
        self.j = j  #AB allignment? JRB: height.
        self.w = w
        self.t = t

    def draw(self, painter, highlight=False):
        pen = self.select_pen(highlight)
        painter.setPen(QColor.fromRgbF(*pen.color))
        font = QFont(self.pen.fontname)

        fontMetrics = QFontMetrics(QFont(self.pen.fontname))
        scale = float(fontMetrics.width(self.t)) / float(self.w)

        if scale < 1.0 or scale > 1.0:
            font.setPointSizeF(font.pointSizeF()/scale);

        painter.setFont(font)#, self.pen.fontsize)) #AB simply setting font size doesn't work
        painter.drawText(
                self.x - self.w / 2.0,
                self.y,
                self.t)
        pass

class EllipseShape(Shape):
    """Used to draw an ellipse shape with a QPainter"""
    def __init__(self, pen, x0, y0, w, h, filled=False):
        Shape.__init__(self)
        self.pen = pen.copy()
        self.x0 = x0
        self.y0 = y0
        self.w = w
        self.h = h
        self.filled = filled

    def draw(self, painter, highlight=False):
        painter.save()
        pen = self.select_pen(highlight)
        if self.filled:
            painter.setPen(QColor.fromRgbF(*pen.fillcolor))
            painter.setBrush(QColor.fromRgbF(*pen.fillcolor))
        else:
            painter.setPen(QPen(QBrush(QColor.fromRgbF(*pen.color)), pen.linewidth, pen.dash))
            painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(self.x0 - self.w, self.y0 - self.h,  self.w * 2,  self.h * 2)
        painter.restore()

class PolygonShape(Shape):
    """Used to draw a polygon with QPainter."""

    def __init__(self, pen, points, filled=False):
        Shape.__init__(self)
        self.pen = pen.copy()
        self.points = points
        self.filled = filled

    def draw(self, painter, highlight=False):

        polygon_points = QPolygonF()
        for x, y in self.points:
            polygon_points.append (QPointF(x, y))

        pen = self.select_pen(highlight)
        painter.save()
        if self.filled:
            painter.setPen(QColor.fromRgbF(*pen.fillcolor))
            painter.setBrush(QColor.fromRgbF(*pen.fillcolor))
        else:
            painter.setPen(QPen(QBrush(QColor.fromRgbF(*pen.color)), pen.linewidth,
                                            pen.dash, Qt.SquareCap, Qt.MiterJoin))
            painter.setBrush(Qt.NoBrush)

        painter.drawPolygon(polygon_points)
        painter.restore()

class LineShape(Shape):
    """Used to draw a line with QPainter."""

    def __init__(self, pen, points):
        Shape.__init__(self)
        self.pen = pen.copy()
        self.points = points

    def draw(self, painter, highlight=False):
        pen = self.select_pen(highlight)
        painter.setPen(QPen(QBrush(QColor.fromRgbF(*pen.color)), pen.linewidth,
                                            pen.dash, Qt.SquareCap, Qt.MiterJoin))

        x0, y0 = self.points[0]
        for x1, y1 in self.points[1:]:
            painter.drawLine(QPointF(x0, y0),  QPointF(x1, y1))
            x0 = x1
            y0 = y1

class BezierShape(Shape):
    """Used to draw a bezier curve with QPainter."""

    def __init__(self, pen, points, filled=False):
        Shape.__init__(self)
        self.pen = pen.copy()
        self.points = points
        self.filled = filled

    def draw(self, painter, highlight=False):
        painter_path = QPainterPath()
        painter_path.moveTo(QPointF(*self.points[0]))
        for i in xrange(1, len(self.points), 3):
            painter_path.cubicTo(
                QPointF(*self.points[i]),
                QPointF(*self.points[i + 1]),
                QPointF(*self.points[i + 2]))
        pen = self.select_pen(highlight)
        qpen = QPen()
        if self.filled:
            brush = QBrush()
            brush.setColor(QColor.fromRgbF(*pen.fillcolor))
            brush.setStyle(Qt.SolidPattern)
            painter.setBrush(brush)
            #qpen.setCapStyle(Qt.RoundCap)
            #qpen.setJoinStyle(Qt.RoundJoin)
            #painter_path.setFillRule(Qt.OddEvenFill)
        else:
            painter.setBrush(Qt.NoBrush)

        qpen.setStyle(pen.dash)
        qpen.setWidth(pen.linewidth)
        qpen.setColor(QColor.fromRgbF(*pen.color))
        painter.setPen(qpen)
        painter.drawPath(painter_path)

class CompoundShape(Shape):
    """Used to draw a set of shapes with QPainter."""

    def __init__(self, shapes):
        Shape.__init__(self)
        self.shapes = shapes

    def draw(self, cr, highlight=False):
        for shape in self.shapes:
            shape.draw(cr, highlight=highlight)

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
#-- Metadata Classes --#

class Url(object):
    """Represents a graphviz URL."""

    def __init__(self, item, url, highlight=None):
        self.item = item
        self.url = url
        if highlight is None:
            highlight = set([item])
        self.highlight = highlight

class Jump(object):
    """Represents a jump to another node's position on the canvas."""

    def __init__(self, item, x, y, highlight=None, url=None):
        self.item = item
        self.x = x
        self.y = y
        if highlight is None:
            highlight = set([item])
        self.highlight = highlight
        self.url = url

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
#-- Graph Representation Classes --#

class Element(CompoundShape):
    """Base class for graph nodes and edges."""

    def __init__(self, shapes):
        CompoundShape.__init__(self, shapes)

    def get_url(self, x, y):
        return None

    def get_jump(self, x, y):
        return None


class Node(Element):
    """An abstract node in the graph, it's spatial location, and it's visual representation."""

    def __init__(self, x, y, w, h, shapes, url):
        Element.__init__(self, shapes)

        self.x = x
        self.y = y

        self.x1 = x - 0.5*w
        self.y1 = y - 0.5*h
        self.x2 = x + 0.5*w
        self.y2 = y + 0.5*h

        self.url = url

    def is_inside(self, x, y):
        """Used to check for 2D-picking via the mouse.
        param x: The x position on the canvas
        param y: The y position on the canvas
        """
        return self.x1 <= x and x <= self.x2 and self.y1 <= y and y <= self.y2

    def get_url(self, x, y):
        """Get the elemnt's metadata."""
        if self.url is None:
            return None
        if self.is_inside(x, y):
            return Url(self, self.url)
        return None

    def get_jump(self, x, y):
        if self.is_inside(x, y):
            return Jump(self, self.x, self.y)
        return None


def square_distance(x1, y1, x2, y2):
    deltax = x2 - x1
    deltay = y2 - y1
    return deltax*deltax + deltay*deltay


class Edge(Element):

    def __init__(self, src, dst, points, shapes, url):
        Element.__init__(self, shapes)
        self.src = src
        self.dst = dst
        self.points = points
        self.url = url

    RADIUS = 10

    def get_jump(self, x, y):
        if square_distance(x, y, *self.points[0]) <= self.RADIUS*self.RADIUS:
            return Jump(self, self.dst.x, self.dst.y, highlight=set([self, self.dst]),url=self.url)
        if square_distance(x, y, *self.points[-1]) <= self.RADIUS*self.RADIUS:
            return Jump(self, self.src.x, self.src.y, highlight=set([self, self.src]),url=self.url)
        return None


#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Graph(Shape):

    def __init__(self, width=1, height=1, shapes=(), nodes=(), edges=(), subgraph_shapes={}):
        Shape.__init__(self)

        self.width = width
        self.height = height
        self.shapes = shapes
        self.nodes = nodes
        self.edges = edges
        self.subgraph_shapes = subgraph_shapes

    def get_size(self):
        return self.width, self.height

    def draw(self, cr, highlight_items=None):
        if highlight_items is None:
            highlight_items = ()
        for shape in self.shapes:
            shape.draw(cr)
        for edge in self.edges:
            edge.draw(cr, highlight=(edge in highlight_items))
        for node in self.nodes:
            node.draw(cr, highlight=(node in highlight_items))

    def get_url(self, x, y):
        for node in self.nodes:
            url = node.get_url(x, y)
            if url is not None:
                return url
        return None

    def get_jump(self, x, y):
        for edge in self.edges:
            jump = edge.get_jump(x, y)
            if jump is not None:
                return jump
        for node in self.nodes:
            jump = node.get_jump(x, y)
            if jump is not None:
                return jump
        return None


#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
class XDotAttrParser:
    """Parser for xdot drawing attributes.
    See also:
    - http://www.graphviz.org/doc/info/output.html#d:xdot
    """

    def __init__(self, parser, buf):
        self.parser = parser
        self.buf = self.unescape(buf)
        self.pos = 0

        self.pen = Pen()
        self.shapes = []

    def __nonzero__(self):
        return self.pos < len(self.buf)

    def unescape(self, buf):
        buf = buf.replace('\\"', '"')
        buf = buf.replace('\\n', '\n')
        return buf

    def read_code(self):
        pos = self.buf.find(" ", self.pos)
        res = self.buf[self.pos:pos]
        self.pos = pos + 1
        while self.pos < len(self.buf) and self.buf[self.pos].isspace():
            self.pos += 1
        return res

    def read_number(self):
        return int(float(self.read_code()))

    def read_float(self):
        return float(self.read_code())

    def read_point(self):
        x = self.read_number()
        y = self.read_number()
        return self.transform(x, y)

    def read_text(self):
        num = self.read_number()
        pos = self.buf.find("-", self.pos) + 1
        self.pos = pos + num
        res = self.buf[pos:self.pos]
        while self.pos < len(self.buf) and self.buf[self.pos].isspace():
            self.pos += 1
        return res

    def read_polygon(self):
        n = self.read_number()
        p = []
#        p = QPointF[]
        for i in range(n):
            x, y = self.read_point()
            p.append((x, y))
#            p.append (QPointF(x, y))
        return p

    def read_color(self):
        # See http://www.graphviz.org/doc/info/attrs.html#k:color
        c = self.read_text()
        c1 = c[:1]
        if c1 == '#':
            hex2float = lambda h: float(int(h, 16)/255.0)
            r = hex2float(c[1:3])
            g = hex2float(c[3:5])
            b = hex2float(c[5:7])
            try:
                a = hex2float(c[7:9])
            except (IndexError, ValueError):
                a = 1.0
            return r, g, b, a
        elif c1.isdigit() or c1 == ".":
            # "H,S,V" or "H S V" or "H, S, V" or any other variation
            h, s, v = map(float, c.replace(",", " ").split())
            r, g, b = colorsys.hsv_to_rgb(h, s, v)
            a = 1.0
            return r, g, b, a
        else:
            return self.lookup_color(c)

    def lookup_color(self, c):
        try:
            color = gtk.gdk.color_parse(c)
        except ValueError:
            pass
        else:
            s = 1.0/65535.0
            r = color.red*s
            g = color.green*s
            b = color.blue*s
            a = 1.0
            return r, g, b, a

        try:
            dummy, scheme, index = c.split('/')
            r, g, b = brewer_colors[scheme][int(index)]
        except (ValueError, KeyError):
            pass
        else:
            s = 1.0/255.0
            r = r*s
            g = g*s
            b = b*s
            a = 1.0
            return r, g, b, a

        sys.stderr.write("unknown color '%s'\n" % c)
        return None

    def parse(self):
        s = self

        while s:
            op = s.read_code()
            if op == "c":
                color = s.read_color()
                if color is not None:
                    self.handle_color(color, filled=False)
            elif op == "C":
                color = s.read_color()
                if color is not None:
                    self.handle_color(color, filled=True)
            elif op == "S":
                # http://www.graphviz.org/doc/info/attrs.html#k:style
                style = s.read_text()
                if style.startswith("setlinewidth("):
                    lw = style.split("(")[1].split(")")[0]
                    lw = float(lw)
                    self.handle_linewidth(lw)
                elif style in ("solid", "dashed"):
                    self.handle_linestyle(style)
            elif op == "F":
                size = s.read_float()
                name = s.read_text()
                self.handle_font(size, name)
            elif op == "T":
                x, y = s.read_point()
                j = s.read_number()
                w = s.read_number()
                t = s.read_text()
                self.handle_text(x, y, j, w, t)
            elif op == "E":
                x0, y0 = s.read_point()
                w = s.read_number()
                h = s.read_number()
                self.handle_ellipse(x0, y0, w, h, filled=True)
            elif op == "e":
                x0, y0 = s.read_point()
                w = s.read_number()
                h = s.read_number()
                self.handle_ellipse(x0, y0, w, h, filled=False)
            elif op == "L":
                points = self.read_polygon()
                self.handle_line(points)
            elif op == "B":
                points = self.read_polygon()
                self.handle_bezier(points, filled=False)
            elif op == "b":
                points = self.read_polygon()
                self.handle_bezier(points, filled=True)
            elif op == "P":
                points = self.read_polygon()
                self.handle_polygon(points, filled=True)
            elif op == "p":
                points = self.read_polygon()
                self.handle_polygon(points, filled=False)
            else:
                sys.stderr.write("unknown xdot opcode '%s'\n" % op)
                break

        return self.shapes

    def transform(self, x, y):
        return self.parser.transform(x, y)

    def handle_color(self, color, filled=False):
        if filled:
            self.pen.fillcolor = color
        else:
            self.pen.color = color

    def handle_linewidth(self, linewidth):
        self.pen.linewidth = linewidth

    def handle_linestyle(self, style):
        if style == "solid":
#            self.pen.dash = ()
            self.pen.dash = Qt.SolidLine
        elif style == "dashed":
#            self.pen.dash = (6, )       # 6pt on, 6pt off
            self.pen.dash = Qt.DashLine

    def handle_font(self, size, name):
        self.pen.fontsize = size
        self.pen.fontname = name

    def handle_text(self, x, y, j, w, t):
        self.shapes.append(TextShape(self.pen, x, y, j, w, t))

    def handle_ellipse(self, x0, y0, w, h, filled=False):
        if filled:
            # xdot uses this to mean "draw a filled shape with an outline"
            self.shapes.append(EllipseShape(self.pen, x0, y0, w, h, filled=True))
        self.shapes.append(EllipseShape(self.pen, x0, y0, w, h))

    def handle_line(self, points):
        self.shapes.append(LineShape(self.pen, points))

    def handle_bezier(self, points, filled=False):
        if filled:
            # xdot uses this to mean "draw a filled shape with an outline"
            self.shapes.append(BezierShape(self.pen, points, filled=True))
        self.shapes.append(BezierShape(self.pen, points))

    def handle_polygon(self, points, filled=False):
        #AB Should filled be handled inside of PolygonShape?
        if filled:
            # xdot uses this to mean "draw a filled shape with an outline"
            self.shapes.append(PolygonShape(self.pen, points, filled=True))
        self.shapes.append(PolygonShape(self.pen, points))


EOF = -1
SKIP = -2


class ParseError(Exception):

    def __init__(self, msg=None, filename=None, line=None, col=None):
        self.msg = msg
        self.filename = filename
        self.line = line
        self.col = col

    def __str__(self):
        return ':'.join([str(part) for part in (self.filename, self.line, self.col, self.msg) if part != None])


#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Scanner:
    """Stateless scanner."""

    # should be overriden by derived classes
    tokens = []
    symbols = {}
    literals = {}
    ignorecase = False

    def __init__(self):
        flags = re.DOTALL
        if self.ignorecase:
            flags |= re.IGNORECASE
        self.tokens_re = re.compile(
            '|'.join(['(' + regexp + ')' for type, regexp, test_lit in self.tokens]),
             flags
        )

    def next(self, buf, pos):
        if pos >= len(buf):
            return EOF, '', pos
        mo = self.tokens_re.match(buf, pos)
        if mo:
            text = mo.group()
            type, regexp, test_lit = self.tokens[mo.lastindex - 1]
            pos = mo.end()
            if test_lit:
                type = self.literals.get(text, type)
            return type, text, pos
        else:
            c = buf[pos]
            return self.symbols.get(c, None), c, pos + 1


class Token:

    def __init__(self, type, text, line, col):
        self.type = type
        self.text = text
        self.line = line
        self.col = col


class Lexer:

    # should be overriden by derived classes
    scanner = None
    tabsize = 8

    newline_re = re.compile(r'\r\n?|\n')

    def __init__(self, buf = None, pos = 0, filename = None, fp = None):
        if fp is not None:
            try:
                fileno = fp.fileno()
                length = os.path.getsize(fp.name)
                import mmap
            except:
                # read whole file into memory
                buf = fp.read()
                pos = 0
            else:
                # map the whole file into memory
                if length:
                    # length must not be zero
                    buf = mmap.mmap(fileno, length, access = mmap.ACCESS_READ)
                    pos = os.lseek(fileno, 0, 1)
                else:
                    buf = ''
                    pos = 0

            if filename is None:
                try:
                    filename = fp.name
                except AttributeError:
                    filename = None

        self.buf = buf
        self.pos = pos
        self.line = 1
        self.col = 1
        self.filename = filename

    def next(self):
        while True:
            # save state
            pos = self.pos
            line = self.line
            col = self.col

            type, text, endpos = self.scanner.next(self.buf, pos)
            assert pos + len(text) == endpos
            self.consume(text)
            type, text = self.filter(type, text)
            self.pos = endpos

            if type == SKIP:
                continue
            elif type is None:
                msg = 'unexpected char '
                if text >= ' ' and text <= '~':
                    msg += "'%s'" % text
                else:
                    msg += "0x%X" % ord(text)
                raise ParseError(msg, self.filename, line, col)
            else:
                break
        return Token(type = type, text = text, line = line, col = col)

    def consume(self, text):
        # update line number
        pos = 0
        for mo in self.newline_re.finditer(text, pos):
            self.line += 1
            self.col = 1
            pos = mo.end()

        # update column number
        while True:
            tabpos = text.find('\t', pos)
            if tabpos == -1:
                break
            self.col += tabpos - pos
            self.col = ((self.col - 1)//self.tabsize + 1)*self.tabsize + 1
            pos = tabpos + 1
        self.col += len(text) - pos


class Parser:

    def __init__(self, lexer):
        self.lexer = lexer
        self.lookahead = self.lexer.next()

    def match(self, type):
        if self.lookahead.type != type:
            raise ParseError(
                msg = 'unexpected token %r' % self.lookahead.text,
                filename = self.lexer.filename,
                line = self.lookahead.line,
                col = self.lookahead.col)

    def skip(self, type):
        while self.lookahead.type != type:
            self.consume()

    def consume(self):
        token = self.lookahead
        self.lookahead = self.lexer.next()
        return token


ID = 0
STR_ID = 1
HTML_ID = 2
EDGE_OP = 3

LSQUARE = 4
RSQUARE = 5
LCURLY = 6
RCURLY = 7
COMMA = 8
COLON = 9
SEMI = 10
EQUAL = 11
PLUS = 12

STRICT = 13
GRAPH = 14
DIGRAPH = 15
NODE = 16
EDGE = 17
SUBGRAPH = 18


class DotScanner(Scanner):

    # token regular expression table
    tokens = [
        # whitespace and comments
        (SKIP,
            r'[ \t\f\r\n\v]+|'
            r'//[^\r\n]*|'
            r'/\*.*?\*/|'
            r'#[^\r\n]*',
        False),

        # Alphanumeric IDs
        (ID, r'[a-zA-Z_\x80-\xff][a-zA-Z0-9_\x80-\xff]*', True),

        # Numeric IDs
        (ID, r'-?(?:\.[0-9]+|[0-9]+(?:\.[0-9]*)?)', False),

        # String IDs
        (STR_ID, r'"[^"\\]*(?:\\.[^"\\]*)*"', False),

        # HTML IDs
        (HTML_ID, r'<[^<>]*(?:<[^<>]*>[^<>]*)*>', False),

        # Edge operators
        (EDGE_OP, r'-[>-]', False),
    ]

    # symbol table
    symbols = {
        '[': LSQUARE,
        ']': RSQUARE,
        '{': LCURLY,
        '}': RCURLY,
        ',': COMMA,
        ':': COLON,
        ';': SEMI,
        '=': EQUAL,
        '+': PLUS,
    }

    # literal table
    literals = {
        'strict': STRICT,
        'graph': GRAPH,
        'digraph': DIGRAPH,
        'node': NODE,
        'edge': EDGE,
        'subgraph': SUBGRAPH,
    }

    ignorecase = True


class DotLexer(Lexer):

    scanner = DotScanner()

    def filter(self, type, text):
        # TODO: handle charset
        if type == STR_ID:
            text = text[1:-1]

            # line continuations
            text = text.replace('\\\r\n', '')
            text = text.replace('\\\r', '')
            text = text.replace('\\\n', '')

            text = text.replace('\\r', '\r')
            text = text.replace('\\n', '\n')
            text = text.replace('\\t', '\t')
            text = text.replace('\\', '')

            type = ID

        elif type == HTML_ID:
            text = text[1:-1]
            type = ID

        return type, text


class DotParser(Parser):

    def __init__(self, lexer):
        Parser.__init__(self, lexer)
        self.graph_attrs = {}
        self.node_attrs = {}
        self.edge_attrs = {}

    def parse(self):
        self.parse_graph()
        self.match(EOF)

    def parse_graph(self):
        if self.lookahead.type == STRICT:
            self.consume()
        self.skip(LCURLY)
        self.consume()
        while self.lookahead.type != RCURLY:
            self.parse_stmt()
        self.consume()

    def parse_subgraph(self):
        id = None
        shapes_before = set(self.shapes)
        if self.lookahead.type == SUBGRAPH:
            self.consume()
            if self.lookahead.type == ID:
                id = self.lookahead.text
                self.consume()
        if self.lookahead.type == LCURLY:
            self.consume()
            while self.lookahead.type != RCURLY:
                self.parse_stmt()
            self.consume()
        new_shapes = set(self.shapes) - shapes_before
        self.subgraph_shapes[id] = [s for s in new_shapes if not any([s in ss for ss in self.subgraph_shapes.values()])]
        return id

    def parse_stmt(self):
        if self.lookahead.type == GRAPH:
            self.consume()
            attrs = self.parse_attrs()
            self.graph_attrs.update(attrs)
            self.handle_graph(attrs)
        elif self.lookahead.type == NODE:
            self.consume()
            self.node_attrs.update(self.parse_attrs())
        elif self.lookahead.type == EDGE:
            self.consume()
            self.edge_attrs.update(self.parse_attrs())
        elif self.lookahead.type in (SUBGRAPH, LCURLY):
            self.parse_subgraph()
        else:
            id = self.parse_node_id()
            if self.lookahead.type == EDGE_OP:
                self.consume()
                node_ids = [id, self.parse_node_id()]
                while self.lookahead.type == EDGE_OP:
                    node_ids.append(self.parse_node_id())
                attrs = self.parse_attrs()
                for i in range(0, len(node_ids) - 1):
                    self.handle_edge(node_ids[i], node_ids[i + 1], attrs)
            elif self.lookahead.type == EQUAL:
                self.consume()
                self.parse_id()
            else:
                attrs = self.parse_attrs()
                self.handle_node(id, attrs)
        if self.lookahead.type == SEMI:
            self.consume()

    def parse_attrs(self):
        attrs = {}
        while self.lookahead.type == LSQUARE:
            self.consume()
            while self.lookahead.type != RSQUARE:
                name, value = self.parse_attr()
                attrs[name] = value
                if self.lookahead.type == COMMA:
                    self.consume()
            self.consume()
        return attrs

    def parse_attr(self):
        name = self.parse_id()
        if self.lookahead.type == EQUAL:
            self.consume()
            value = self.parse_id()
        else:
            value = 'true'
        return name, value

    def parse_node_id(self):
        node_id = self.parse_id()
        if self.lookahead.type == COLON:
            self.consume()
            port = self.parse_id()
            if self.lookahead.type == COLON:
                self.consume()
                compass_pt = self.parse_id()
            else:
                compass_pt = None
        else:
            port = None
            compass_pt = None
        # XXX: we don't really care about port and compass point values when parsing xdot
        return node_id

    def parse_id(self):
        self.match(ID)
        id = self.lookahead.text
        self.consume()
        return id

    def handle_graph(self, attrs):
        pass

    def handle_node(self, id, attrs):
        pass

    def handle_edge(self, src_id, dst_id, attrs):
        pass


class XDotParser(DotParser):

    def __init__(self, xdotcode):
        lexer = DotLexer(buf = xdotcode)
        DotParser.__init__(self, lexer)

        self.nodes = []
        self.edges = []
        self.shapes = []
        self.node_by_name = {}
        self.top_graph = True
        self.subgraph_shapes = {}

    def handle_graph(self, attrs):
        if self.top_graph:
            try:
                bb = attrs['bb']
            except KeyError:
                return

            if not bb:
                return

            xmin, ymin, xmax, ymax = map(float, bb.split(","))

            self.xoffset = -xmin
            self.yoffset = -ymax
            self.xscale = 1.0
            self.yscale = -1.0
            # FIXME: scale from points to pixels

            self.width = xmax - xmin
            self.height = ymax - ymin

            self.top_graph = False

        for attr in ("_draw_", "_ldraw_", "_hdraw_", "_tdraw_", "_hldraw_", "_tldraw_"):
            if attr in attrs:
                parser = XDotAttrParser(self, attrs[attr])
                self.shapes.extend(parser.parse())

    def handle_node(self, id, attrs):
        try:
            pos = attrs['pos']
        except KeyError:
            return

        x, y = self.parse_node_pos(pos)
        w = float(attrs['width'])*72
        h = float(attrs['height'])*72
        shapes = []
        for attr in ("_draw_", "_ldraw_"):
            if attr in attrs:
                parser = XDotAttrParser(self, attrs[attr])
                shapes.extend(parser.parse())
        url = attrs.get('URL', None)
        node = Node(x, y, w, h, shapes, url)
        self.node_by_name[id] = node
        if shapes:
            self.nodes.append(node)

    def handle_edge(self, src_id, dst_id, attrs):
        try:
            pos = attrs['pos']
        except KeyError:
            return

        points = self.parse_edge_pos(pos)
        shapes = []
        for attr in ("_draw_", "_ldraw_", "_hdraw_", "_tdraw_", "_hldraw_", "_tldraw_"):
            if attr in attrs:
                parser = XDotAttrParser(self, attrs[attr])
                shapes.extend(parser.parse())
        url = attrs.get('URL', None)
        if shapes:
            src = self.node_by_name[src_id]
            dst = self.node_by_name[dst_id]
            self.edges.append(Edge(src, dst, points, shapes, url))

    def parse(self):
        DotParser.parse(self)

        """
        for k,shapes in self.subgraph_shapes.iteritems():
          self.shapes += shapes
        """

        return Graph(self.width, self.height, self.shapes, self.nodes, self.edges, self.subgraph_shapes)

    def parse_node_pos(self, pos):
        x, y = pos.split(",")
        return self.transform(float(x), float(y))

    def parse_edge_pos(self, pos):
        points = []
        for entry in pos.split(' '):
            fields = entry.split(',')
            try:
                x, y = fields
            except ValueError:
                # TODO: handle start/end points
                #AB Handle data like like e,40,50 here
                continue
            else:
                points.append(self.transform(float(x), float(y)))
        return points

    def transform(self, x, y):
        # XXX: this is not the right place for this code
        x = (x + self.xoffset)*self.xscale
        y = (y + self.yoffset)*self.yscale
        return x, y


#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Animation(object):

    step = 0.03 # seconds

    def __init__(self, dot_widget):
        self.dot_widget = dot_widget
        self.timeout_id = None

    def start(self):
        self.timeout_id = QTimer();
        self.timeout_id.start(int(self.step * 1000))

    def stop(self):
        self.dot_widget.animation = NoAnimation(self.dot_widget)
        if self.timeout_id is not None:
            self.timeout_id.stop()
            self.timeout_id = None

    def tick(self):
        self.stop()


class NoAnimation(Animation):

    def start(self):
        pass

    def stop(self):
        pass


class LinearAnimation(Animation):

    duration = 0.6

    def start(self):
        self.started = time.time()
        Animation.start(self)

    def tick(self):
        t = (time.time() - self.started) / self.duration
        self.animate(max(0, min(t, 1)))
#        return (t < 1) #AB returning False stops the timer
        if t >= 1: #AB
            self.timeout_id.stop() #AB

    def animate(self, t):
        pass


class MoveToAnimation(LinearAnimation):

    def __init__(self, dot_widget, target_x, target_y):
        Animation.__init__(self, dot_widget)
        self.source_x = dot_widget.x
        self.source_y = dot_widget.y
        self.target_x = target_x
        self.target_y = target_y

    def animate(self, t):
        sx, sy = self.source_x, self.source_y
        tx, ty = self.target_x, self.target_y
        self.dot_widget.x = tx * t + sx * (1-t)
        self.dot_widget.y = ty * t + sy * (1-t)
        self.dot_widget.update()

class ZoomToAnimation(MoveToAnimation):

    def __init__(self, dot_widget, target_x, target_y):
        MoveToAnimation.__init__(self, dot_widget, target_x, target_y)
        self.source_zoom = dot_widget.zoom_ratio
        self.target_zoom = self.source_zoom
        self.extra_zoom = 0

        middle_zoom = 0.5 * (self.source_zoom + self.target_zoom)

        distance = math.hypot(self.source_x - self.target_x,
                              self.source_y - self.target_y)
#        rect = self.dot_widget.get_allocation()
        rect = self.dot_widget.rect()
#        visible = min(rect.width, rect.height) / self.dot_widget.zoom_ratio
        visible = min(rect.width(), rect.height()) / self.dot_widget.zoom_ratio
        visible *= 0.9
        if distance > 0:
            desired_middle_zoom = visible / distance
            self.extra_zoom = min(0, 4 * (desired_middle_zoom - middle_zoom))

    def animate(self, t):
        a, b, c = self.source_zoom, self.extra_zoom, self.target_zoom
        self.dot_widget.zoom_ratio = c*t + b*t*(1-t) + a*(1-t)
        self.dot_widget.zoom_to_fit_on_resize = False
        MoveToAnimation.animate(self, t)

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
class DragAction(object):

    def __init__(self, dot_widget):
        self.dot_widget = dot_widget

    def on_button_press(self, event):
#        self.startmousex = self.prevmousex = event.x
        self.startmousex = self.prevmousex = event.x()
        self.startmousey = self.prevmousey = event.y()
        self.start()

    def on_motion_notify(self, event):
        deltax = self.prevmousex - event.x()
        deltay = self.prevmousey - event.y()
        self.drag(deltax, deltay)
        self.prevmousex = event.x()
        self.prevmousey = event.y()

    def on_button_release(self, event):
        self.stopmousex = event.x()
        self.stopmousey = event.y()
        self.stop()

    def draw(self, cr):
        pass

    def start(self):
        pass

    def drag(self, deltax, deltay):
        pass

    def stop(self):
        pass

    def abort(self):
        pass


class NullAction(DragAction):

    def on_motion_notify(self, event):
        x, y = event.x(), event.y()

        dot_widget = self.dot_widget
        item = dot_widget.get_url(x, y)
        if item is None:
            item = dot_widget.get_jump(x, y)
        if item is not None:
            dot_widget.setCursor(Qt.PointingHandCursor)
            dot_widget.set_highlight(item.highlight)
        else:
            dot_widget.setCursor(Qt.ArrowCursor)
            dot_widget.set_highlight(None)


class PanAction(DragAction):

    def start(self):
        self.dot_widget.setCursor(Qt.ClosedHandCursor)

    def drag(self, deltax, deltay):
        self.dot_widget.x += deltax / self.dot_widget.zoom_ratio
        self.dot_widget.y += deltay / self.dot_widget.zoom_ratio
        self.dot_widget.update()

    def stop(self):
        self.dot_widget.cursor().setShape(Qt.ArrowCursor)

    abort = stop


class ZoomAction(DragAction):

    def drag(self, deltax, deltay):
        self.dot_widget.zoom_ratio *= 1.005 ** (deltax + deltay)
        self.dot_widget.zoom_to_fit_on_resize = False
        self.dot_widget.update()

def stop(self):
        self.dot_widget.update()


class ZoomAreaAction(DragAction):

    def drag(self, deltax, deltay):
        self.dot_widget.update()

    def draw(self, painter):
        #TODO: implement this for qt
        print "ERROR: UNIMPLEMENTED ZoomAreaAction.draw"
        return
        painter.save()
        painter.set_source_rgba(.5, .5, 1.0, 0.25)
        painter.rectangle(self.startmousex, self.startmousey,
                     self.prevmousex - self.startmousex,
                     self.prevmousey - self.startmousey)
        painter.fill()
        painter.set_source_rgba(.5, .5, 1.0, 1.0)
        painter.set_line_width(1)
        painter.rectangle(self.startmousex - .5, self.startmousey - .5,
                     self.prevmousex - self.startmousex + 1,
                     self.prevmousey - self.startmousey + 1)
        painter.stroke()
        painter.restore()

    def stop(self):
        x1, y1 = self.dot_widget.window_to_graph(self.startmousex,
                                              self.startmousey)
        x2, y2 = self.dot_widget.window_to_graph(self.stopmousex,
                                              self.stopmousey)
        self.dot_widget.zoom_to_area(x1, y1, x2, y2)

    def abort(self):
        self.dot_widget.update()
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
class DotWidget(QWidget):
#class DotWidget(gtk.DrawingArea):
    """Qt widget that draws dot graphs."""

    filter = 'dot'

    def __init__(self,  parent=None):
        super(DotWidget,  self).__init__(parent)

        # qt version flag
        self.qt4_version = False
        if QT_BINDING_VERSION.startswith('4'):
            self.qt4_version = True

        self.graph = Graph()
        self.openfilename = None

#        self.set_flags(gtk.CAN_FOCUS)
#        self.add_events(gtk.gdk.BUTTON_PRESS_MASK | gtk.gdk.BUTTON_RELEASE_MASK)
#        self.connect("button-press-event", self.on_area_button_press)
#        self.connect("button-release-event", self.on_area_button_release)
#        self.add_events(gtk.gdk.POINTER_MOTION_MASK | gtk.gdk.POINTER_MOTION_HINT_MASK | gtk.gdk.BUTTON_RELEASE_MASK)
#        self.connect("motion-notify-event", self.on_area_motion_notify)
#        self.connect("scroll-event", self.on_area_scroll_event)
#        self.connect("size-allocate", self.on_area_size_allocate)
#        self.connect('key-press-event', self.on_key_press_event)

        self.x, self.y = 0.0, 0.0
        self.zoom_ratio = 1.0
        self.zoom_to_fit_on_resize = False
        self.animation = NoAnimation(self)
        self.drag_action = NullAction(self)
        self.presstime = None
        self.highlight = None

        # Callback register
        self.select_cbs = []
        self.dc = None
        self.ctx = None
        self.items_by_url = {}

        self.setMouseTracking (True) # track all mouse events

    ZOOM_INCREMENT = 1.25
    ZOOM_TO_FIT_MARGIN = 12
    POS_INCREMENT = 100

    ### User callbacks
    def register_select_callback(self, cb):
        self.select_cbs.append(cb)

    def set_filter(self, filter):
        self.filter = filter

    def set_dotcode(self, dotcode, filename='<stdin>',center=True):
        if isinstance(dotcode, unicode):
            dotcode = dotcode.encode('utf8')
        p = subprocess.Popen(
            [self.filter, '-Txdot'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=False,
            universal_newlines=True
        )
        xdotcode, error = p.communicate(dotcode)
        if p.returncode != 0:
            print "UNABLE TO SHELL TO DOT", error
#            dialog = gtk.MessageDialog(type=gtk.MESSAGE_ERROR,
#                                       message_format=error,
#                                       buttons=gtk.BUTTONS_OK)
#            dialog.set_title('Dot Viewer')
#            dialog.run()
#            dialog.destroy()
            return False
        try:
            self.set_xdotcode(xdotcode,center)

            # Store references to all the items
            self.items_by_url = {}
            for item in self.graph.nodes + self.graph.edges:
                if item.url is not None:
                    self.items_by_url[item.url] = item

            # Store references to subgraph states
            self.subgraph_shapes = self.graph.subgraph_shapes

        except ParseError, ex:
#            dialog = gtk.MessageDialog(type=gtk.MESSAGE_ERROR,
#                                       message_format=str(ex),
#                                       buttons=gtk.BUTTONS_OK)
#            dialog.set_title('Dot Viewer')
#            dialog.run()
#            dialog.destroy()
            return False
        else:
            self.openfilename = filename
            return True

    def set_xdotcode(self, xdotcode, center=True):
        #print xdotcode
        parser = XDotParser(xdotcode)
        self.graph = parser.parse()
        # zoom out and center image everytime a new graph is set
        # self.zoom_image(self.zoom_ratio, center=center)

    def reload(self):
        if self.openfilename is not None:
            try:
                fp = open(str(self.openfilename), "rb")
                self.set_dotcode(fp.read(), self.openfilename)
                fp.close()
            except IOError:
                pass

    def paintEvent (self,  event=None):
#    def do_expose_event(self, event):
#        cr = self.window.cairo_create()
        painter = QPainter (self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.TextAntialiasing)
        painter.setRenderHint(QPainter.HighQualityAntialiasing)

        # set a clip region for the expose event
#        cr.rectangle(
#            event.area.x, event.area.y,
#            event.area.width, event.area.height
#        )
#        cr.clip()
#
#        cr.set_source_rgba(1.0, 1.0, 1.0, 1.0)
#        cr.paint()

        painter.setClipping(True)
        painter.setClipRect(self.rect())
        painter.setBackground(QBrush(Qt.blue,Qt.SolidPattern))
        painter.save()

#        rect = self.get_allocation()
        rect = self.rect() # JRB was self.rect()
#        cr.translate(0.5*rect.width, 0.5*rect.height)
        painter.translate(0.5*rect.width(), 0.5*rect.height())
        painter.scale(self.zoom_ratio, self.zoom_ratio)
        painter.translate(-self.x, -self.y)

        self.graph.draw(painter, highlight_items=self.highlight)
        painter.restore()

        self.drag_action.draw(painter)


    def get_current_pos(self):
        return self.x, self.y

#AB This function is not used
    def set_current_pos(self, x, y):
        self.x = x
        self.y = y
        self.update()

    def set_highlight(self, items):
        if self.highlight != items:
            self.highlight = items
            self.update()

    def zoom_image(self, zoom_ratio, center=False, pos=None):
        if center:
            self.x = self.graph.width/2
            self.y = self.graph.height/2
        elif pos is not None:
#            rect = self.get_allocation()
            rect = self.rect()
            x, y = pos
#            x -= 0.5*rect.width
            x -= 0.5*rect.width()
#            y -= 0.5*rect.height
            y -= 0.5*rect.height()
            self.x += x / self.zoom_ratio - x / zoom_ratio
            self.y += y / self.zoom_ratio - y / zoom_ratio
        self.zoom_ratio = zoom_ratio
        self.zoom_to_fit_on_resize = False
        self.update()

    def zoom_to_area(self, x1, y1, x2, y2):
#        rect = self.get_allocation()
        rect = self.rect()
        width = abs(x1 - x2)
        height = abs(y1 - y2)
        self.zoom_ratio = min(
            float(rect.width())/float(width),
            float(rect.height())/float(height)
        )
        self.zoom_to_fit_on_resize = False
        self.x = (x1 + x2) / 2
        self.y = (y1 + y2) / 2
        self.update()

    def zoom_to_fit(self):
#        rect = self.get_allocation()
        rect = self.rect()
#        rect.x += self.ZOOM_TO_FIT_MARGIN
        rect.setX (rect.x() + self.ZOOM_TO_FIT_MARGIN)
#        rect.y += self.ZOOM_TO_FIT_MARGIN
        rect.setY (rect.y() + self.ZOOM_TO_FIT_MARGIN)
#        rect.width -= 2 * self.ZOOM_TO_FIT_MARGIN
        rect.setWidth(rect.width() - 2 * self.ZOOM_TO_FIT_MARGIN)
#        rect.height -= 2 * self.ZOOM_TO_FIT_MARGIN
        rect.setHeight(rect.height() - 2 * self.ZOOM_TO_FIT_MARGIN)
        zoom_ratio = min(
#            float(rect.width)/float(self.graph.width),
            float(rect.width())/float(self.graph.width),
#            float(rect.height)/float(self.graph.height)
            float(rect.height())/float(self.graph.height)
        )
        self.zoom_image(zoom_ratio, center=True)
        self.zoom_to_fit_on_resize = True

    def on_zoom_in(self):
#    def on_zoom_in(self, action):
        self.zoom_image(self.zoom_ratio * self.ZOOM_INCREMENT)

    def on_zoom_out(self):
#    def on_zoom_out(self, action):
        self.zoom_image(self.zoom_ratio / self.ZOOM_INCREMENT)

    def on_zoom_fit(self):
#    def on_zoom_fit(self, action):
        self.zoom_to_fit()

    def on_zoom_100(self):
#    def on_zoom_100(self, action):
        self.zoom_image(1.0)

    def keyPressEvent(self, event):
        self.animation.stop()
        self.drag_action.abort()
        if event.key() == Qt.Key_Left:
            self.x -= self.POS_INCREMENT/self.zoom_ratio
            self.update()
        elif event.key() == Qt.Key_Right:
            self.x += self.POS_INCREMENT/self.zoom_ratio
            self.update()
        elif event.key() == Qt.Key_Up:
            self.y -= self.POS_INCREMENT/self.zoom_ratio
            self.update()
        elif event.key() == Qt.Key_Down:
            self.y += self.POS_INCREMENT/self.zoom_ratio
            self.update()
        elif event.key() == Qt.Key_PageUp:
            self.zoom_image(self.zoom_ratio * self.ZOOM_INCREMENT)
            self.update()
        elif event.key() == Qt.Key_PageDown:
            self.zoom_image(self.zoom_ratio / self.ZOOM_INCREMENT)
            self.update()
        elif event.key() == Qt.Key_PageUp:
            self.drag_action.abort()
            self.drag_action = NullAction(self)
        elif event.key() == Qt.Key_R:
            self.reload()
        elif event.key() == Qt.Key_F:
            self.zoom_to_fit()
        event.accept()

    def get_drag_action(self, event):
        modifiers = event.modifiers()
        if event.button() in (Qt.LeftButton, Qt.MidButton):
            if modifiers & Qt.ControlModifier:
                return ZoomAction
            elif modifiers & Qt.ShiftModifier:
                return ZoomAreaAction
            else:
                return PanAction
        return NullAction

    def mousePressEvent(self, event):
        self.animation.stop()
        self.drag_action.abort()

        for cb in self.select_cbs:
            cb(event)

        action_type = self.get_drag_action(event)
        self.drag_action = action_type(self)
        self.drag_action.on_button_press(event)

        self.presstime = time.time()
        self.pressx = event.x()
        self.pressy = event.y()
        event.accept()

    def is_click(self, event, click_fuzz=4, click_timeout=1.0):
        if self.presstime is None:
            # got a button release without seeing the press?
            return False
        # XXX instead of doing this complicated logic, shouldn't we listen
        # for gtk's clicked event instead?
        deltax = self.pressx - event.x()
        deltay = self.pressy - event.y()
        return (time.time() < self.presstime + click_timeout
                and math.hypot(deltax, deltay) < click_fuzz)

    def mouseReleaseEvent(self, event):
        self.drag_action.on_button_release(event)
        self.drag_action = NullAction(self)
        if event.button() == Qt.LeftButton and self.is_click(event):
            x, y = event.x(), event.y()
            url = self.get_url(x, y)
            if url is None:
                jump = self.get_jump(x, y)
                if jump is not None:
                    self.animate_to(jump.x, jump.y)

            event.accept()
            return
        if event.button() == Qt.RightButton and self.is_click(event):
            x, y = event.x(), event.y()
            url = self.get_url(x, y)
            if url is None:
                jump = self.get_jump(x, y)
                if jump is not None:
                    self.animate_to(jump.x, jump.y)

        if event.button() in (Qt.LeftButton, Qt.MidButton):
            event.accept()
        return

    def on_area_scroll_event(self, area, event):
        return False

    def wheelEvent(self, event):
        if self.qt4_version:
            # PyQt4
            if event.delta() > 0:
                self.zoom_image(self.zoom_ratio * self.ZOOM_INCREMENT,
                            pos=(event.x(), event.y()))
            if event.delta() < 0:
                self.zoom_image(self.zoom_ratio / self.ZOOM_INCREMENT,
                            pos=(event.x(), event.y()))
        else:
            # PyQt5
            if event.angleDelta().y() > 0:
                self.zoom_image(self.zoom_ratio * self.ZOOM_INCREMENT,
                            pos=(event.x(), event.y()))
            if event.angleDelta().y() < 0:
                self.zoom_image(self.zoom_ratio / self.ZOOM_INCREMENT,
                            pos=(event.x(), event.y()))

    def mouseMoveEvent(self, event):
        self.drag_action.on_motion_notify(event)
        self.setFocus()
        for cb in self.select_cbs:
            cb(event)

    def on_area_size_allocate(self, area, allocation):
        if self.zoom_to_fit_on_resize:
            self.zoom_to_fit()

    def animate_to(self, x, y):
        self.animation = ZoomToAnimation(self, x, y)
        self.animation.start()

    def window_to_graph(self, x, y):
#        rect = self.get_allocation()
        rect = self.rect()
        x -= 0.5*rect.width()
        y -= 0.5*rect.height()
        x /= self.zoom_ratio
        y /= self.zoom_ratio
        x += self.x
        y += self.y
        return x, y

    def get_url(self, x, y):
        x, y = self.window_to_graph(x, y)
        return self.graph.get_url(x, y)

    def get_jump(self, x, y):
        x, y = self.window_to_graph(x, y)
        return self.graph.get_jump(x, y)


#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
#class DotWindow(gtk.Window):
class DotWindow(QMainWindow):

    def __init__(self):
        super(DotWindow,  self).__init__(None)
        self.graph = Graph()
        self.setWindowTitle(QApplication.applicationName())
        self.widget = DotWidget()
        self.widget.setContextMenuPolicy(Qt.ActionsContextMenu)
        self.setCentralWidget(self.widget)

        palette = QPalette ()
        palette.setColor(QPalette.Background, Qt.white)
        self.setPalette(palette)

        self.filename = None

        file_open_action = self.create_action("&Open...", self.on_open,
                QKeySequence.Open, "fileopen", "Open an existing dot file")
        file_reload_action = self.create_action("&Refresh", self.on_reload,
                QKeySequence.Refresh, "view-refresh", "Reload opened dot file")
        zoom_in_action = self.create_action("Zoom In", self.widget.on_zoom_in,
                QKeySequence.ZoomIn, "zoom-in", "Zoom in")
        zoom_out_action = self.create_action("Zoom Out", self.widget.on_zoom_out,
                QKeySequence.ZoomIn, "zoom-out", "Zoom Out")
        zoom_fit_action = self.create_action("Zoom Fit", self.widget.on_zoom_fit,
                None, "zoom-fit-best", "Zoom Fit")
        zoom_100_action = self.create_action("Zoom 100%", self.widget.on_zoom_100,
                None, "zoom-original", "Zoom 100%")

        self.file_menu = self.menuBar().addMenu("&File")
        self.file_menu_actions = (file_open_action, file_reload_action)
        self.connect(self.file_menu, SIGNAL("aboutToShow()"), self.update_file_menu)

        file_toolbar = self.addToolBar("File")
        file_toolbar.setObjectName("FileToolBar")
        self.add_actions(file_toolbar, (file_open_action, file_reload_action))

        fileToolbar = self.addToolBar("Zoom")
        fileToolbar.setObjectName("ZoomToolBar")
        self.add_actions(fileToolbar, (zoom_in_action, zoom_out_action,  zoom_fit_action,  zoom_100_action))

        settings = QSettings()
        self.recent_files = settings.value("RecentFiles").toStringList()
        size = settings.value("MainWindow/Size", QVariant(QSize(512, 512))).toSize()
        self.resize(size)
        position = settings.value("MainWindow/Position", QVariant(QPoint(0, 0))).toPoint()
        self.move(position)

        self.restoreState(settings.value("MainWindow/State").toByteArray())
        self.update_file_menu()

        self.show()

    def create_action(self, text, slot=None, shortcut=None, icon=None,
                     tip=None, checkable=False, signal="triggered()"):
        action = QAction(text, self)
        if icon is not None:
            action.setIcon(QIcon.fromTheme(icon))
        if shortcut is not None:
            action.setShortcut(shortcut)
        if tip is not None:
            action.setToolTip(tip)
            action.setStatusTip(tip)
        if slot is not None:
            self.connect(action, SIGNAL(signal), slot)
        if checkable:
            action.setCheckable(True)
        return action

    def add_actions(self, target, actions):
        for action in actions:
            if action is None:
                target.addSeparator()
            else:
                target.addAction(action)

    def update_file(self, filename):
        import os
        if not hasattr(self, "last_mtime"):
            self.last_mtime = None

        current_mtime = os.stat(filename).st_mtime
        if current_mtime != self.last_mtime:
            self.last_mtime = current_mtime
            self.open_file(filename)

        return True

    def set_filter(self, filter):
        self.widget.set_filter(filter)

    def set_dotcode(self, dotcode, filename='<stdin>'):
        if self.widget.set_dotcode(dotcode, filename):
            self.setWindowTitle(os.path.basename(filename) + ' - ' + QApplication.applicationName())
            self.widget.zoom_to_fit()

    def set_xdotcode(self, xdotcode, filename='<stdin>'):
        if self.widget.set_xdotcode(xdotcode):
            self.setWindowTitle(os.path.basename(filename) + ' - ' + QApplication.applicationName())
            self.widget.zoom_to_fit()

    def open_file(self, filename=None):
        if filename is None:
            action = self.sender()
            if isinstance(action, QAction):
                filename = unicode(action.data().toString())
            else:
                return
        try:
            fp = file(filename, 'rt')
            self.set_dotcode(fp.read(), filename)
            fp.close()
            self.add_recent_file(filename)
        except IOError, ex:
            pass

    def on_open(self):
        dir = os.path.dirname(self.filename) \
                if self.filename is not None else "."
        formats = ["*.dot"]
        filename = unicode(QFileDialog.getOpenFileName(self,
                            "Open dot File", dir,
                            "Dot files (%s)" % " ".join(formats)))
        if filename:
            self.open_file(filename)

    def on_reload(self):
        self.widget.reload()

    def update_file_menu(self):
        self.file_menu.clear()
        self.add_actions(self.file_menu, self.file_menu_actions[:-1])
        current = QString(self.filename) \
                if self.filename is not None else None
        recent_files = []
        for fname in self.recent_files:
            if fname != current and QFile.exists(fname):
                recent_files.append(fname)
        if recent_files:
            self.file_menu.addSeparator()
            for i, fname in enumerate(recent_files):
                action = QAction(QIcon(":/icon.png"), "&%d %s" % (i + 1, QFileInfo(fname).fileName()), self)
                action.setData(QVariant(fname))
                self.connect(action, SIGNAL("triggered()"), self.open_file)
                self.file_menu.addAction(action)
        self.file_menu.addSeparator()
        self.file_menu.addAction(self.file_menu_actions[-1])

    def add_recent_file(self, filename):
        if filename is None:
            return
        if not self.recent_files.contains(filename):
            self.recent_files.prepend(QString(filename))
            while self.recent_files.count() > 14:
                self.recent_files.takeLast()

def closeEvent(self, event):
        settings = QSettings()
        filename = QVariant(QString(self.filename)) if self.filename is not None else QVariant()
        settings.setValue("LastFile", filename)
        recent_files = QVariant(self.recent_files) if self.recent_files else QVariant()
        settings.setValue("RecentFiles", recent_files)
        settings.setValue("MainWindow/Size", QVariant(self.size()))

        #AB It looks like PyQt 4.7.4 has a bug that results in both self.pos() and self.geometry() returning the same values
        # see http://doc.qt.nokia.com/latest/application-windows.html#window-geometry for explanation of why they should be different
        # The results is a small (5,24) shift of window on consequent start from previous position
        #print (self.pos())
        #print (self.geometry())
        settings.setValue("MainWindow/Position", QVariant(self.pos()))
        #settings.setValue("MainWindow/Geometry", QVariant(self.saveGeometry()))
        settings.setValue("MainWindow/State", QVariant(self.saveState()))

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------
def main():
    import optparse

    parser = optparse.OptionParser(usage='\n\t%prog [file]', version='%%prog %s' % __version__)
    # This program can shell to different Graphviz filter processes specified by -f option
    parser.add_option(
        '-f', '--filter',
        type='choice', choices=('dot', 'neato', 'twopi', 'circo', 'fdp'),
        dest='filter', default='dot',
        help='graphviz filter: dot, neato, twopi, circo, or fdp [default: %default]')

    (options, args) = parser.parse_args(sys.argv[1:])
    if len(args) > 1:
        parser.error('incorrect number of arguments')

    app = QApplication(sys.argv)
    app.setOrganizationName("RobotNV")
    app.setOrganizationDomain("robotNV.com")
    app.setApplicationName("Dot Viewer")
    app.setWindowIcon(QIcon(":/icon.png"))

    win = DotWindow()
    win.show()
#    win.connect('destroy', gtk.main_quit)
#    win.set_filter(options.filter)
    if len(args) >= 1:
        if args[0] == '-':
            win.set_dotcode(sys.stdin.read())
        else:
            win.open_file(args[0])
#            gobject.timeout_add(1000, win.update_file, args[0])
#    gtk.main()
    sys.exit(app.exec_())




# Apache-Style Software License for ColorBrewer software and ColorBrewer Color
# Schemes, Version 1.1
#
# Copyright (c) 2002 Cynthia Brewer, Mark Harrower, and The Pennsylvania State
# University. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions as source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#    2. The end-user documentation included with the redistribution, if any,
#    must include the following acknowledgment:
#
#       This product includes color specifications and designs developed by
#       Cynthia Brewer (http://colorbrewer.org/).
#
#    Alternately, this acknowledgment may appear in the software itself, if and
#    wherever such third-party acknowledgments normally appear.
#
#    3. The name "ColorBrewer" must not be used to endorse or promote products
#    derived from this software without prior written permission. For written
#    permission, please contact Cynthia Brewer at cbrewer@psu.edu.
#
#    4. Products derived from this software may not be called "ColorBrewer",
#    nor may "ColorBrewer" appear in their name, without prior written
#    permission of Cynthia Brewer.
#
# THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL CYNTHIA
# BREWER, MARK HARROWER, OR THE PENNSYLVANIA STATE UNIVERSITY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
brewer_colors = {
    'accent3': [(127, 201, 127), (190, 174, 212), (253, 192, 134)],
    'accent4': [(127, 201, 127), (190, 174, 212), (253, 192, 134), (255, 255, 153)],
    'accent5': [(127, 201, 127), (190, 174, 212), (253, 192, 134), (255, 255, 153), (56, 108, 176)],
    'accent6': [(127, 201, 127), (190, 174, 212), (253, 192, 134), (255, 255, 153), (56, 108, 176), (240, 2, 127)],
    'accent7': [(127, 201, 127), (190, 174, 212), (253, 192, 134), (255, 255, 153), (56, 108, 176), (240, 2, 127), (191, 91, 23)],
    'accent8': [(127, 201, 127), (190, 174, 212), (253, 192, 134), (255, 255, 153), (56, 108, 176), (240, 2, 127), (191, 91, 23), (102, 102, 102)],
    'blues3': [(222, 235, 247), (158, 202, 225), (49, 130, 189)],
    'blues4': [(239, 243, 255), (189, 215, 231), (107, 174, 214), (33, 113, 181)],
    'blues5': [(239, 243, 255), (189, 215, 231), (107, 174, 214), (49, 130, 189), (8, 81, 156)],
    'blues6': [(239, 243, 255), (198, 219, 239), (158, 202, 225), (107, 174, 214), (49, 130, 189), (8, 81, 156)],
    'blues7': [(239, 243, 255), (198, 219, 239), (158, 202, 225), (107, 174, 214), (66, 146, 198), (33, 113, 181), (8, 69, 148)],
    'blues8': [(247, 251, 255), (222, 235, 247), (198, 219, 239), (158, 202, 225), (107, 174, 214), (66, 146, 198), (33, 113, 181), (8, 69, 148)],
    'blues9': [(247, 251, 255), (222, 235, 247), (198, 219, 239), (158, 202, 225), (107, 174, 214), (66, 146, 198), (33, 113, 181), (8, 81, 156), (8, 48, 107)],
    'brbg10': [(84, 48, 5), (0, 60, 48), (140, 81, 10), (191, 129, 45), (223, 194, 125), (246, 232, 195), (199, 234, 229), (128, 205, 193), (53, 151, 143), (1, 102, 94)],
    'brbg11': [(84, 48, 5), (1, 102, 94), (0, 60, 48), (140, 81, 10), (191, 129, 45), (223, 194, 125), (246, 232, 195), (245, 245, 245), (199, 234, 229), (128, 205, 193), (53, 151, 143)],
    'brbg3': [(216, 179, 101), (245, 245, 245), (90, 180, 172)],
    'brbg4': [(166, 97, 26), (223, 194, 125), (128, 205, 193), (1, 133, 113)],
    'brbg5': [(166, 97, 26), (223, 194, 125), (245, 245, 245), (128, 205, 193), (1, 133, 113)],
    'brbg6': [(140, 81, 10), (216, 179, 101), (246, 232, 195), (199, 234, 229), (90, 180, 172), (1, 102, 94)],
    'brbg7': [(140, 81, 10), (216, 179, 101), (246, 232, 195), (245, 245, 245), (199, 234, 229), (90, 180, 172), (1, 102, 94)],
    'brbg8': [(140, 81, 10), (191, 129, 45), (223, 194, 125), (246, 232, 195), (199, 234, 229), (128, 205, 193), (53, 151, 143), (1, 102, 94)],
    'brbg9': [(140, 81, 10), (191, 129, 45), (223, 194, 125), (246, 232, 195), (245, 245, 245), (199, 234, 229), (128, 205, 193), (53, 151, 143), (1, 102, 94)],
    'bugn3': [(229, 245, 249), (153, 216, 201), (44, 162, 95)],
    'bugn4': [(237, 248, 251), (178, 226, 226), (102, 194, 164), (35, 139, 69)],
    'bugn5': [(237, 248, 251), (178, 226, 226), (102, 194, 164), (44, 162, 95), (0, 109, 44)],
    'bugn6': [(237, 248, 251), (204, 236, 230), (153, 216, 201), (102, 194, 164), (44, 162, 95), (0, 109, 44)],
    'bugn7': [(237, 248, 251), (204, 236, 230), (153, 216, 201), (102, 194, 164), (65, 174, 118), (35, 139, 69), (0, 88, 36)],
    'bugn8': [(247, 252, 253), (229, 245, 249), (204, 236, 230), (153, 216, 201), (102, 194, 164), (65, 174, 118), (35, 139, 69), (0, 88, 36)],
    'bugn9': [(247, 252, 253), (229, 245, 249), (204, 236, 230), (153, 216, 201), (102, 194, 164), (65, 174, 118), (35, 139, 69), (0, 109, 44), (0, 68, 27)],
    'bupu3': [(224, 236, 244), (158, 188, 218), (136, 86, 167)],
    'bupu4': [(237, 248, 251), (179, 205, 227), (140, 150, 198), (136, 65, 157)],
    'bupu5': [(237, 248, 251), (179, 205, 227), (140, 150, 198), (136, 86, 167), (129, 15, 124)],
    'bupu6': [(237, 248, 251), (191, 211, 230), (158, 188, 218), (140, 150, 198), (136, 86, 167), (129, 15, 124)],
    'bupu7': [(237, 248, 251), (191, 211, 230), (158, 188, 218), (140, 150, 198), (140, 107, 177), (136, 65, 157), (110, 1, 107)],
    'bupu8': [(247, 252, 253), (224, 236, 244), (191, 211, 230), (158, 188, 218), (140, 150, 198), (140, 107, 177), (136, 65, 157), (110, 1, 107)],
    'bupu9': [(247, 252, 253), (224, 236, 244), (191, 211, 230), (158, 188, 218), (140, 150, 198), (140, 107, 177), (136, 65, 157), (129, 15, 124), (77, 0, 75)],
    'dark23': [(27, 158, 119), (217, 95, 2), (117, 112, 179)],
    'dark24': [(27, 158, 119), (217, 95, 2), (117, 112, 179), (231, 41, 138)],
    'dark25': [(27, 158, 119), (217, 95, 2), (117, 112, 179), (231, 41, 138), (102, 166, 30)],
    'dark26': [(27, 158, 119), (217, 95, 2), (117, 112, 179), (231, 41, 138), (102, 166, 30), (230, 171, 2)],
    'dark27': [(27, 158, 119), (217, 95, 2), (117, 112, 179), (231, 41, 138), (102, 166, 30), (230, 171, 2), (166, 118, 29)],
    'dark28': [(27, 158, 119), (217, 95, 2), (117, 112, 179), (231, 41, 138), (102, 166, 30), (230, 171, 2), (166, 118, 29), (102, 102, 102)],
    'gnbu3': [(224, 243, 219), (168, 221, 181), (67, 162, 202)],
    'gnbu4': [(240, 249, 232), (186, 228, 188), (123, 204, 196), (43, 140, 190)],
    'gnbu5': [(240, 249, 232), (186, 228, 188), (123, 204, 196), (67, 162, 202), (8, 104, 172)],
    'gnbu6': [(240, 249, 232), (204, 235, 197), (168, 221, 181), (123, 204, 196), (67, 162, 202), (8, 104, 172)],
    'gnbu7': [(240, 249, 232), (204, 235, 197), (168, 221, 181), (123, 204, 196), (78, 179, 211), (43, 140, 190), (8, 88, 158)],
    'gnbu8': [(247, 252, 240), (224, 243, 219), (204, 235, 197), (168, 221, 181), (123, 204, 196), (78, 179, 211), (43, 140, 190), (8, 88, 158)],
    'gnbu9': [(247, 252, 240), (224, 243, 219), (204, 235, 197), (168, 221, 181), (123, 204, 196), (78, 179, 211), (43, 140, 190), (8, 104, 172), (8, 64, 129)],
    'greens3': [(229, 245, 224), (161, 217, 155), (49, 163, 84)],
    'greens4': [(237, 248, 233), (186, 228, 179), (116, 196, 118), (35, 139, 69)],
    'greens5': [(237, 248, 233), (186, 228, 179), (116, 196, 118), (49, 163, 84), (0, 109, 44)],
    'greens6': [(237, 248, 233), (199, 233, 192), (161, 217, 155), (116, 196, 118), (49, 163, 84), (0, 109, 44)],
    'greens7': [(237, 248, 233), (199, 233, 192), (161, 217, 155), (116, 196, 118), (65, 171, 93), (35, 139, 69), (0, 90, 50)],
    'greens8': [(247, 252, 245), (229, 245, 224), (199, 233, 192), (161, 217, 155), (116, 196, 118), (65, 171, 93), (35, 139, 69), (0, 90, 50)],
    'greens9': [(247, 252, 245), (229, 245, 224), (199, 233, 192), (161, 217, 155), (116, 196, 118), (65, 171, 93), (35, 139, 69), (0, 109, 44), (0, 68, 27)],
    'greys3': [(240, 240, 240), (189, 189, 189), (99, 99, 99)],
    'greys4': [(247, 247, 247), (204, 204, 204), (150, 150, 150), (82, 82, 82)],
    'greys5': [(247, 247, 247), (204, 204, 204), (150, 150, 150), (99, 99, 99), (37, 37, 37)],
    'greys6': [(247, 247, 247), (217, 217, 217), (189, 189, 189), (150, 150, 150), (99, 99, 99), (37, 37, 37)],
    'greys7': [(247, 247, 247), (217, 217, 217), (189, 189, 189), (150, 150, 150), (115, 115, 115), (82, 82, 82), (37, 37, 37)],
    'greys8': [(255, 255, 255), (240, 240, 240), (217, 217, 217), (189, 189, 189), (150, 150, 150), (115, 115, 115), (82, 82, 82), (37, 37, 37)],
    'greys9': [(255, 255, 255), (240, 240, 240), (217, 217, 217), (189, 189, 189), (150, 150, 150), (115, 115, 115), (82, 82, 82), (37, 37, 37), (0, 0, 0)],
    'oranges3': [(254, 230, 206), (253, 174, 107), (230, 85, 13)],
    'oranges4': [(254, 237, 222), (253, 190, 133), (253, 141, 60), (217, 71, 1)],
    'oranges5': [(254, 237, 222), (253, 190, 133), (253, 141, 60), (230, 85, 13), (166, 54, 3)],
    'oranges6': [(254, 237, 222), (253, 208, 162), (253, 174, 107), (253, 141, 60), (230, 85, 13), (166, 54, 3)],
    'oranges7': [(254, 237, 222), (253, 208, 162), (253, 174, 107), (253, 141, 60), (241, 105, 19), (217, 72, 1), (140, 45, 4)],
    'oranges8': [(255, 245, 235), (254, 230, 206), (253, 208, 162), (253, 174, 107), (253, 141, 60), (241, 105, 19), (217, 72, 1), (140, 45, 4)],
    'oranges9': [(255, 245, 235), (254, 230, 206), (253, 208, 162), (253, 174, 107), (253, 141, 60), (241, 105, 19), (217, 72, 1), (166, 54, 3), (127, 39, 4)],
    'orrd3': [(254, 232, 200), (253, 187, 132), (227, 74, 51)],
    'orrd4': [(254, 240, 217), (253, 204, 138), (252, 141, 89), (215, 48, 31)],
    'orrd5': [(254, 240, 217), (253, 204, 138), (252, 141, 89), (227, 74, 51), (179, 0, 0)],
    'orrd6': [(254, 240, 217), (253, 212, 158), (253, 187, 132), (252, 141, 89), (227, 74, 51), (179, 0, 0)],
    'orrd7': [(254, 240, 217), (253, 212, 158), (253, 187, 132), (252, 141, 89), (239, 101, 72), (215, 48, 31), (153, 0, 0)],
    'orrd8': [(255, 247, 236), (254, 232, 200), (253, 212, 158), (253, 187, 132), (252, 141, 89), (239, 101, 72), (215, 48, 31), (153, 0, 0)],
    'orrd9': [(255, 247, 236), (254, 232, 200), (253, 212, 158), (253, 187, 132), (252, 141, 89), (239, 101, 72), (215, 48, 31), (179, 0, 0), (127, 0, 0)],
    'paired10': [(166, 206, 227), (106, 61, 154), (31, 120, 180), (178, 223, 138), (51, 160, 44), (251, 154, 153), (227, 26, 28), (253, 191, 111), (255, 127, 0), (202, 178, 214)],
    'paired11': [(166, 206, 227), (106, 61, 154), (255, 255, 153), (31, 120, 180), (178, 223, 138), (51, 160, 44), (251, 154, 153), (227, 26, 28), (253, 191, 111), (255, 127, 0), (202, 178, 214)],
    'paired12': [(166, 206, 227), (106, 61, 154), (255, 255, 153), (177, 89, 40), (31, 120, 180), (178, 223, 138), (51, 160, 44), (251, 154, 153), (227, 26, 28), (253, 191, 111), (255, 127, 0), (202, 178, 214)],
    'paired3': [(166, 206, 227), (31, 120, 180), (178, 223, 138)],
    'paired4': [(166, 206, 227), (31, 120, 180), (178, 223, 138), (51, 160, 44)],
    'paired5': [(166, 206, 227), (31, 120, 180), (178, 223, 138), (51, 160, 44), (251, 154, 153)],
    'paired6': [(166, 206, 227), (31, 120, 180), (178, 223, 138), (51, 160, 44), (251, 154, 153), (227, 26, 28)],
    'paired7': [(166, 206, 227), (31, 120, 180), (178, 223, 138), (51, 160, 44), (251, 154, 153), (227, 26, 28), (253, 191, 111)],
    'paired8': [(166, 206, 227), (31, 120, 180), (178, 223, 138), (51, 160, 44), (251, 154, 153), (227, 26, 28), (253, 191, 111), (255, 127, 0)],
    'paired9': [(166, 206, 227), (31, 120, 180), (178, 223, 138), (51, 160, 44), (251, 154, 153), (227, 26, 28), (253, 191, 111), (255, 127, 0), (202, 178, 214)],
    'pastel13': [(251, 180, 174), (179, 205, 227), (204, 235, 197)],
    'pastel14': [(251, 180, 174), (179, 205, 227), (204, 235, 197), (222, 203, 228)],
    'pastel15': [(251, 180, 174), (179, 205, 227), (204, 235, 197), (222, 203, 228), (254, 217, 166)],
    'pastel16': [(251, 180, 174), (179, 205, 227), (204, 235, 197), (222, 203, 228), (254, 217, 166), (255, 255, 204)],
    'pastel17': [(251, 180, 174), (179, 205, 227), (204, 235, 197), (222, 203, 228), (254, 217, 166), (255, 255, 204), (229, 216, 189)],
    'pastel18': [(251, 180, 174), (179, 205, 227), (204, 235, 197), (222, 203, 228), (254, 217, 166), (255, 255, 204), (229, 216, 189), (253, 218, 236)],
    'pastel19': [(251, 180, 174), (179, 205, 227), (204, 235, 197), (222, 203, 228), (254, 217, 166), (255, 255, 204), (229, 216, 189), (253, 218, 236), (242, 242, 242)],
    'pastel23': [(179, 226, 205), (253, 205, 172), (203, 213, 232)],
    'pastel24': [(179, 226, 205), (253, 205, 172), (203, 213, 232), (244, 202, 228)],
    'pastel25': [(179, 226, 205), (253, 205, 172), (203, 213, 232), (244, 202, 228), (230, 245, 201)],
    'pastel26': [(179, 226, 205), (253, 205, 172), (203, 213, 232), (244, 202, 228), (230, 245, 201), (255, 242, 174)],
    'pastel27': [(179, 226, 205), (253, 205, 172), (203, 213, 232), (244, 202, 228), (230, 245, 201), (255, 242, 174), (241, 226, 204)],
    'pastel28': [(179, 226, 205), (253, 205, 172), (203, 213, 232), (244, 202, 228), (230, 245, 201), (255, 242, 174), (241, 226, 204), (204, 204, 204)],
    'piyg10': [(142, 1, 82), (39, 100, 25), (197, 27, 125), (222, 119, 174), (241, 182, 218), (253, 224, 239), (230, 245, 208), (184, 225, 134), (127, 188, 65), (77, 146, 33)],
    'piyg11': [(142, 1, 82), (77, 146, 33), (39, 100, 25), (197, 27, 125), (222, 119, 174), (241, 182, 218), (253, 224, 239), (247, 247, 247), (230, 245, 208), (184, 225, 134), (127, 188, 65)],
    'piyg3': [(233, 163, 201), (247, 247, 247), (161, 215, 106)],
    'piyg4': [(208, 28, 139), (241, 182, 218), (184, 225, 134), (77, 172, 38)],
    'piyg5': [(208, 28, 139), (241, 182, 218), (247, 247, 247), (184, 225, 134), (77, 172, 38)],
    'piyg6': [(197, 27, 125), (233, 163, 201), (253, 224, 239), (230, 245, 208), (161, 215, 106), (77, 146, 33)],
    'piyg7': [(197, 27, 125), (233, 163, 201), (253, 224, 239), (247, 247, 247), (230, 245, 208), (161, 215, 106), (77, 146, 33)],
    'piyg8': [(197, 27, 125), (222, 119, 174), (241, 182, 218), (253, 224, 239), (230, 245, 208), (184, 225, 134), (127, 188, 65), (77, 146, 33)],
    'piyg9': [(197, 27, 125), (222, 119, 174), (241, 182, 218), (253, 224, 239), (247, 247, 247), (230, 245, 208), (184, 225, 134), (127, 188, 65), (77, 146, 33)],
    'prgn10': [(64, 0, 75), (0, 68, 27), (118, 42, 131), (153, 112, 171), (194, 165, 207), (231, 212, 232), (217, 240, 211), (166, 219, 160), (90, 174, 97), (27, 120, 55)],
    'prgn11': [(64, 0, 75), (27, 120, 55), (0, 68, 27), (118, 42, 131), (153, 112, 171), (194, 165, 207), (231, 212, 232), (247, 247, 247), (217, 240, 211), (166, 219, 160), (90, 174, 97)],
    'prgn3': [(175, 141, 195), (247, 247, 247), (127, 191, 123)],
    'prgn4': [(123, 50, 148), (194, 165, 207), (166, 219, 160), (0, 136, 55)],
    'prgn5': [(123, 50, 148), (194, 165, 207), (247, 247, 247), (166, 219, 160), (0, 136, 55)],
    'prgn6': [(118, 42, 131), (175, 141, 195), (231, 212, 232), (217, 240, 211), (127, 191, 123), (27, 120, 55)],
    'prgn7': [(118, 42, 131), (175, 141, 195), (231, 212, 232), (247, 247, 247), (217, 240, 211), (127, 191, 123), (27, 120, 55)],
    'prgn8': [(118, 42, 131), (153, 112, 171), (194, 165, 207), (231, 212, 232), (217, 240, 211), (166, 219, 160), (90, 174, 97), (27, 120, 55)],
    'prgn9': [(118, 42, 131), (153, 112, 171), (194, 165, 207), (231, 212, 232), (247, 247, 247), (217, 240, 211), (166, 219, 160), (90, 174, 97), (27, 120, 55)],
    'pubu3': [(236, 231, 242), (166, 189, 219), (43, 140, 190)],
    'pubu4': [(241, 238, 246), (189, 201, 225), (116, 169, 207), (5, 112, 176)],
    'pubu5': [(241, 238, 246), (189, 201, 225), (116, 169, 207), (43, 140, 190), (4, 90, 141)],
    'pubu6': [(241, 238, 246), (208, 209, 230), (166, 189, 219), (116, 169, 207), (43, 140, 190), (4, 90, 141)],
    'pubu7': [(241, 238, 246), (208, 209, 230), (166, 189, 219), (116, 169, 207), (54, 144, 192), (5, 112, 176), (3, 78, 123)],
    'pubu8': [(255, 247, 251), (236, 231, 242), (208, 209, 230), (166, 189, 219), (116, 169, 207), (54, 144, 192), (5, 112, 176), (3, 78, 123)],
    'pubu9': [(255, 247, 251), (236, 231, 242), (208, 209, 230), (166, 189, 219), (116, 169, 207), (54, 144, 192), (5, 112, 176), (4, 90, 141), (2, 56, 88)],
    'pubugn3': [(236, 226, 240), (166, 189, 219), (28, 144, 153)],
    'pubugn4': [(246, 239, 247), (189, 201, 225), (103, 169, 207), (2, 129, 138)],
    'pubugn5': [(246, 239, 247), (189, 201, 225), (103, 169, 207), (28, 144, 153), (1, 108, 89)],
    'pubugn6': [(246, 239, 247), (208, 209, 230), (166, 189, 219), (103, 169, 207), (28, 144, 153), (1, 108, 89)],
    'pubugn7': [(246, 239, 247), (208, 209, 230), (166, 189, 219), (103, 169, 207), (54, 144, 192), (2, 129, 138), (1, 100, 80)],
    'pubugn8': [(255, 247, 251), (236, 226, 240), (208, 209, 230), (166, 189, 219), (103, 169, 207), (54, 144, 192), (2, 129, 138), (1, 100, 80)],
    'pubugn9': [(255, 247, 251), (236, 226, 240), (208, 209, 230), (166, 189, 219), (103, 169, 207), (54, 144, 192), (2, 129, 138), (1, 108, 89), (1, 70, 54)],
    'puor10': [(127, 59, 8), (45, 0, 75), (179, 88, 6), (224, 130, 20), (253, 184, 99), (254, 224, 182), (216, 218, 235), (178, 171, 210), (128, 115, 172), (84, 39, 136)],
    'puor11': [(127, 59, 8), (84, 39, 136), (45, 0, 75), (179, 88, 6), (224, 130, 20), (253, 184, 99), (254, 224, 182), (247, 247, 247), (216, 218, 235), (178, 171, 210), (128, 115, 172)],
    'puor3': [(241, 163, 64), (247, 247, 247), (153, 142, 195)],
    'puor4': [(230, 97, 1), (253, 184, 99), (178, 171, 210), (94, 60, 153)],
    'puor5': [(230, 97, 1), (253, 184, 99), (247, 247, 247), (178, 171, 210), (94, 60, 153)],
    'puor6': [(179, 88, 6), (241, 163, 64), (254, 224, 182), (216, 218, 235), (153, 142, 195), (84, 39, 136)],
    'puor7': [(179, 88, 6), (241, 163, 64), (254, 224, 182), (247, 247, 247), (216, 218, 235), (153, 142, 195), (84, 39, 136)],
    'puor8': [(179, 88, 6), (224, 130, 20), (253, 184, 99), (254, 224, 182), (216, 218, 235), (178, 171, 210), (128, 115, 172), (84, 39, 136)],
    'puor9': [(179, 88, 6), (224, 130, 20), (253, 184, 99), (254, 224, 182), (247, 247, 247), (216, 218, 235), (178, 171, 210), (128, 115, 172), (84, 39, 136)],
    'purd3': [(231, 225, 239), (201, 148, 199), (221, 28, 119)],
    'purd4': [(241, 238, 246), (215, 181, 216), (223, 101, 176), (206, 18, 86)],
    'purd5': [(241, 238, 246), (215, 181, 216), (223, 101, 176), (221, 28, 119), (152, 0, 67)],
    'purd6': [(241, 238, 246), (212, 185, 218), (201, 148, 199), (223, 101, 176), (221, 28, 119), (152, 0, 67)],
    'purd7': [(241, 238, 246), (212, 185, 218), (201, 148, 199), (223, 101, 176), (231, 41, 138), (206, 18, 86), (145, 0, 63)],
    'purd8': [(247, 244, 249), (231, 225, 239), (212, 185, 218), (201, 148, 199), (223, 101, 176), (231, 41, 138), (206, 18, 86), (145, 0, 63)],
    'purd9': [(247, 244, 249), (231, 225, 239), (212, 185, 218), (201, 148, 199), (223, 101, 176), (231, 41, 138), (206, 18, 86), (152, 0, 67), (103, 0, 31)],
    'purples3': [(239, 237, 245), (188, 189, 220), (117, 107, 177)],
    'purples4': [(242, 240, 247), (203, 201, 226), (158, 154, 200), (106, 81, 163)],
    'purples5': [(242, 240, 247), (203, 201, 226), (158, 154, 200), (117, 107, 177), (84, 39, 143)],
    'purples6': [(242, 240, 247), (218, 218, 235), (188, 189, 220), (158, 154, 200), (117, 107, 177), (84, 39, 143)],
    'purples7': [(242, 240, 247), (218, 218, 235), (188, 189, 220), (158, 154, 200), (128, 125, 186), (106, 81, 163), (74, 20, 134)],
    'purples8': [(252, 251, 253), (239, 237, 245), (218, 218, 235), (188, 189, 220), (158, 154, 200), (128, 125, 186), (106, 81, 163), (74, 20, 134)],
    'purples9': [(252, 251, 253), (239, 237, 245), (218, 218, 235), (188, 189, 220), (158, 154, 200), (128, 125, 186), (106, 81, 163), (84, 39, 143), (63, 0, 125)],
    'rdbu10': [(103, 0, 31), (5, 48, 97), (178, 24, 43), (214, 96, 77), (244, 165, 130), (253, 219, 199), (209, 229, 240), (146, 197, 222), (67, 147, 195), (33, 102, 172)],
    'rdbu11': [(103, 0, 31), (33, 102, 172), (5, 48, 97), (178, 24, 43), (214, 96, 77), (244, 165, 130), (253, 219, 199), (247, 247, 247), (209, 229, 240), (146, 197, 222), (67, 147, 195)],
    'rdbu3': [(239, 138, 98), (247, 247, 247), (103, 169, 207)],
    'rdbu4': [(202, 0, 32), (244, 165, 130), (146, 197, 222), (5, 113, 176)],
    'rdbu5': [(202, 0, 32), (244, 165, 130), (247, 247, 247), (146, 197, 222), (5, 113, 176)],
    'rdbu6': [(178, 24, 43), (239, 138, 98), (253, 219, 199), (209, 229, 240), (103, 169, 207), (33, 102, 172)],
    'rdbu7': [(178, 24, 43), (239, 138, 98), (253, 219, 199), (247, 247, 247), (209, 229, 240), (103, 169, 207), (33, 102, 172)],
    'rdbu8': [(178, 24, 43), (214, 96, 77), (244, 165, 130), (253, 219, 199), (209, 229, 240), (146, 197, 222), (67, 147, 195), (33, 102, 172)],
    'rdbu9': [(178, 24, 43), (214, 96, 77), (244, 165, 130), (253, 219, 199), (247, 247, 247), (209, 229, 240), (146, 197, 222), (67, 147, 195), (33, 102, 172)],
    'rdgy10': [(103, 0, 31), (26, 26, 26), (178, 24, 43), (214, 96, 77), (244, 165, 130), (253, 219, 199), (224, 224, 224), (186, 186, 186), (135, 135, 135), (77, 77, 77)],
    'rdgy11': [(103, 0, 31), (77, 77, 77), (26, 26, 26), (178, 24, 43), (214, 96, 77), (244, 165, 130), (253, 219, 199), (255, 255, 255), (224, 224, 224), (186, 186, 186), (135, 135, 135)],
    'rdgy3': [(239, 138, 98), (255, 255, 255), (153, 153, 153)],
    'rdgy4': [(202, 0, 32), (244, 165, 130), (186, 186, 186), (64, 64, 64)],
    'rdgy5': [(202, 0, 32), (244, 165, 130), (255, 255, 255), (186, 186, 186), (64, 64, 64)],
    'rdgy6': [(178, 24, 43), (239, 138, 98), (253, 219, 199), (224, 224, 224), (153, 153, 153), (77, 77, 77)],
    'rdgy7': [(178, 24, 43), (239, 138, 98), (253, 219, 199), (255, 255, 255), (224, 224, 224), (153, 153, 153), (77, 77, 77)],
    'rdgy8': [(178, 24, 43), (214, 96, 77), (244, 165, 130), (253, 219, 199), (224, 224, 224), (186, 186, 186), (135, 135, 135), (77, 77, 77)],
    'rdgy9': [(178, 24, 43), (214, 96, 77), (244, 165, 130), (253, 219, 199), (255, 255, 255), (224, 224, 224), (186, 186, 186), (135, 135, 135), (77, 77, 77)],
    'rdpu3': [(253, 224, 221), (250, 159, 181), (197, 27, 138)],
    'rdpu4': [(254, 235, 226), (251, 180, 185), (247, 104, 161), (174, 1, 126)],
    'rdpu5': [(254, 235, 226), (251, 180, 185), (247, 104, 161), (197, 27, 138), (122, 1, 119)],
    'rdpu6': [(254, 235, 226), (252, 197, 192), (250, 159, 181), (247, 104, 161), (197, 27, 138), (122, 1, 119)],
    'rdpu7': [(254, 235, 226), (252, 197, 192), (250, 159, 181), (247, 104, 161), (221, 52, 151), (174, 1, 126), (122, 1, 119)],
    'rdpu8': [(255, 247, 243), (253, 224, 221), (252, 197, 192), (250, 159, 181), (247, 104, 161), (221, 52, 151), (174, 1, 126), (122, 1, 119)],
    'rdpu9': [(255, 247, 243), (253, 224, 221), (252, 197, 192), (250, 159, 181), (247, 104, 161), (221, 52, 151), (174, 1, 126), (122, 1, 119), (73, 0, 106)],
    'rdylbu10': [(165, 0, 38), (49, 54, 149), (215, 48, 39), (244, 109, 67), (253, 174, 97), (254, 224, 144), (224, 243, 248), (171, 217, 233), (116, 173, 209), (69, 117, 180)],
    'rdylbu11': [(165, 0, 38), (69, 117, 180), (49, 54, 149), (215, 48, 39), (244, 109, 67), (253, 174, 97), (254, 224, 144), (255, 255, 191), (224, 243, 248), (171, 217, 233), (116, 173, 209)],
    'rdylbu3': [(252, 141, 89), (255, 255, 191), (145, 191, 219)],
    'rdylbu4': [(215, 25, 28), (253, 174, 97), (171, 217, 233), (44, 123, 182)],
    'rdylbu5': [(215, 25, 28), (253, 174, 97), (255, 255, 191), (171, 217, 233), (44, 123, 182)],
    'rdylbu6': [(215, 48, 39), (252, 141, 89), (254, 224, 144), (224, 243, 248), (145, 191, 219), (69, 117, 180)],
    'rdylbu7': [(215, 48, 39), (252, 141, 89), (254, 224, 144), (255, 255, 191), (224, 243, 248), (145, 191, 219), (69, 117, 180)],
    'rdylbu8': [(215, 48, 39), (244, 109, 67), (253, 174, 97), (254, 224, 144), (224, 243, 248), (171, 217, 233), (116, 173, 209), (69, 117, 180)],
    'rdylbu9': [(215, 48, 39), (244, 109, 67), (253, 174, 97), (254, 224, 144), (255, 255, 191), (224, 243, 248), (171, 217, 233), (116, 173, 209), (69, 117, 180)],
    'rdylgn10': [(165, 0, 38), (0, 104, 55), (215, 48, 39), (244, 109, 67), (253, 174, 97), (254, 224, 139), (217, 239, 139), (166, 217, 106), (102, 189, 99), (26, 152, 80)],
    'rdylgn11': [(165, 0, 38), (26, 152, 80), (0, 104, 55), (215, 48, 39), (244, 109, 67), (253, 174, 97), (254, 224, 139), (255, 255, 191), (217, 239, 139), (166, 217, 106), (102, 189, 99)],
    'rdylgn3': [(252, 141, 89), (255, 255, 191), (145, 207, 96)],
    'rdylgn4': [(215, 25, 28), (253, 174, 97), (166, 217, 106), (26, 150, 65)],
    'rdylgn5': [(215, 25, 28), (253, 174, 97), (255, 255, 191), (166, 217, 106), (26, 150, 65)],
    'rdylgn6': [(215, 48, 39), (252, 141, 89), (254, 224, 139), (217, 239, 139), (145, 207, 96), (26, 152, 80)],
    'rdylgn7': [(215, 48, 39), (252, 141, 89), (254, 224, 139), (255, 255, 191), (217, 239, 139), (145, 207, 96), (26, 152, 80)],
    'rdylgn8': [(215, 48, 39), (244, 109, 67), (253, 174, 97), (254, 224, 139), (217, 239, 139), (166, 217, 106), (102, 189, 99), (26, 152, 80)],
    'rdylgn9': [(215, 48, 39), (244, 109, 67), (253, 174, 97), (254, 224, 139), (255, 255, 191), (217, 239, 139), (166, 217, 106), (102, 189, 99), (26, 152, 80)],
    'reds3': [(254, 224, 210), (252, 146, 114), (222, 45, 38)],
    'reds4': [(254, 229, 217), (252, 174, 145), (251, 106, 74), (203, 24, 29)],
    'reds5': [(254, 229, 217), (252, 174, 145), (251, 106, 74), (222, 45, 38), (165, 15, 21)],
    'reds6': [(254, 229, 217), (252, 187, 161), (252, 146, 114), (251, 106, 74), (222, 45, 38), (165, 15, 21)],
    'reds7': [(254, 229, 217), (252, 187, 161), (252, 146, 114), (251, 106, 74), (239, 59, 44), (203, 24, 29), (153, 0, 13)],
    'reds8': [(255, 245, 240), (254, 224, 210), (252, 187, 161), (252, 146, 114), (251, 106, 74), (239, 59, 44), (203, 24, 29), (153, 0, 13)],
    'reds9': [(255, 245, 240), (254, 224, 210), (252, 187, 161), (252, 146, 114), (251, 106, 74), (239, 59, 44), (203, 24, 29), (165, 15, 21), (103, 0, 13)],
    'set13': [(228, 26, 28), (55, 126, 184), (77, 175, 74)],
    'set14': [(228, 26, 28), (55, 126, 184), (77, 175, 74), (152, 78, 163)],
    'set15': [(228, 26, 28), (55, 126, 184), (77, 175, 74), (152, 78, 163), (255, 127, 0)],
    'set16': [(228, 26, 28), (55, 126, 184), (77, 175, 74), (152, 78, 163), (255, 127, 0), (255, 255, 51)],
    'set17': [(228, 26, 28), (55, 126, 184), (77, 175, 74), (152, 78, 163), (255, 127, 0), (255, 255, 51), (166, 86, 40)],
    'set18': [(228, 26, 28), (55, 126, 184), (77, 175, 74), (152, 78, 163), (255, 127, 0), (255, 255, 51), (166, 86, 40), (247, 129, 191)],
    'set19': [(228, 26, 28), (55, 126, 184), (77, 175, 74), (152, 78, 163), (255, 127, 0), (255, 255, 51), (166, 86, 40), (247, 129, 191), (153, 153, 153)],
    'set23': [(102, 194, 165), (252, 141, 98), (141, 160, 203)],
    'set24': [(102, 194, 165), (252, 141, 98), (141, 160, 203), (231, 138, 195)],
    'set25': [(102, 194, 165), (252, 141, 98), (141, 160, 203), (231, 138, 195), (166, 216, 84)],
    'set26': [(102, 194, 165), (252, 141, 98), (141, 160, 203), (231, 138, 195), (166, 216, 84), (255, 217, 47)],
    'set27': [(102, 194, 165), (252, 141, 98), (141, 160, 203), (231, 138, 195), (166, 216, 84), (255, 217, 47), (229, 196, 148)],
    'set28': [(102, 194, 165), (252, 141, 98), (141, 160, 203), (231, 138, 195), (166, 216, 84), (255, 217, 47), (229, 196, 148), (179, 179, 179)],
    'set310': [(141, 211, 199), (188, 128, 189), (255, 255, 179), (190, 186, 218), (251, 128, 114), (128, 177, 211), (253, 180, 98), (179, 222, 105), (252, 205, 229), (217, 217, 217)],
    'set311': [(141, 211, 199), (188, 128, 189), (204, 235, 197), (255, 255, 179), (190, 186, 218), (251, 128, 114), (128, 177, 211), (253, 180, 98), (179, 222, 105), (252, 205, 229), (217, 217, 217)],
    'set312': [(141, 211, 199), (188, 128, 189), (204, 235, 197), (255, 237, 111), (255, 255, 179), (190, 186, 218), (251, 128, 114), (128, 177, 211), (253, 180, 98), (179, 222, 105), (252, 205, 229), (217, 217, 217)],
    'set33': [(141, 211, 199), (255, 255, 179), (190, 186, 218)],
    'set34': [(141, 211, 199), (255, 255, 179), (190, 186, 218), (251, 128, 114)],
    'set35': [(141, 211, 199), (255, 255, 179), (190, 186, 218), (251, 128, 114), (128, 177, 211)],
    'set36': [(141, 211, 199), (255, 255, 179), (190, 186, 218), (251, 128, 114), (128, 177, 211), (253, 180, 98)],
    'set37': [(141, 211, 199), (255, 255, 179), (190, 186, 218), (251, 128, 114), (128, 177, 211), (253, 180, 98), (179, 222, 105)],
    'set38': [(141, 211, 199), (255, 255, 179), (190, 186, 218), (251, 128, 114), (128, 177, 211), (253, 180, 98), (179, 222, 105), (252, 205, 229)],
    'set39': [(141, 211, 199), (255, 255, 179), (190, 186, 218), (251, 128, 114), (128, 177, 211), (253, 180, 98), (179, 222, 105), (252, 205, 229), (217, 217, 217)],
    'spectral10': [(158, 1, 66), (94, 79, 162), (213, 62, 79), (244, 109, 67), (253, 174, 97), (254, 224, 139), (230, 245, 152), (171, 221, 164), (102, 194, 165), (50, 136, 189)],
    'spectral11': [(158, 1, 66), (50, 136, 189), (94, 79, 162), (213, 62, 79), (244, 109, 67), (253, 174, 97), (254, 224, 139), (255, 255, 191), (230, 245, 152), (171, 221, 164), (102, 194, 165)],
    'spectral3': [(252, 141, 89), (255, 255, 191), (153, 213, 148)],
    'spectral4': [(215, 25, 28), (253, 174, 97), (171, 221, 164), (43, 131, 186)],
    'spectral5': [(215, 25, 28), (253, 174, 97), (255, 255, 191), (171, 221, 164), (43, 131, 186)],
    'spectral6': [(213, 62, 79), (252, 141, 89), (254, 224, 139), (230, 245, 152), (153, 213, 148), (50, 136, 189)],
    'spectral7': [(213, 62, 79), (252, 141, 89), (254, 224, 139), (255, 255, 191), (230, 245, 152), (153, 213, 148), (50, 136, 189)],
    'spectral8': [(213, 62, 79), (244, 109, 67), (253, 174, 97), (254, 224, 139), (230, 245, 152), (171, 221, 164), (102, 194, 165), (50, 136, 189)],
    'spectral9': [(213, 62, 79), (244, 109, 67), (253, 174, 97), (254, 224, 139), (255, 255, 191), (230, 245, 152), (171, 221, 164), (102, 194, 165), (50, 136, 189)],
    'ylgn3': [(247, 252, 185), (173, 221, 142), (49, 163, 84)],
    'ylgn4': [(255, 255, 204), (194, 230, 153), (120, 198, 121), (35, 132, 67)],
    'ylgn5': [(255, 255, 204), (194, 230, 153), (120, 198, 121), (49, 163, 84), (0, 104, 55)],
    'ylgn6': [(255, 255, 204), (217, 240, 163), (173, 221, 142), (120, 198, 121), (49, 163, 84), (0, 104, 55)],
    'ylgn7': [(255, 255, 204), (217, 240, 163), (173, 221, 142), (120, 198, 121), (65, 171, 93), (35, 132, 67), (0, 90, 50)],
    'ylgn8': [(255, 255, 229), (247, 252, 185), (217, 240, 163), (173, 221, 142), (120, 198, 121), (65, 171, 93), (35, 132, 67), (0, 90, 50)],
    'ylgn9': [(255, 255, 229), (247, 252, 185), (217, 240, 163), (173, 221, 142), (120, 198, 121), (65, 171, 93), (35, 132, 67), (0, 104, 55), (0, 69, 41)],
    'ylgnbu3': [(237, 248, 177), (127, 205, 187), (44, 127, 184)],
    'ylgnbu4': [(255, 255, 204), (161, 218, 180), (65, 182, 196), (34, 94, 168)],
    'ylgnbu5': [(255, 255, 204), (161, 218, 180), (65, 182, 196), (44, 127, 184), (37, 52, 148)],
    'ylgnbu6': [(255, 255, 204), (199, 233, 180), (127, 205, 187), (65, 182, 196), (44, 127, 184), (37, 52, 148)],
    'ylgnbu7': [(255, 255, 204), (199, 233, 180), (127, 205, 187), (65, 182, 196), (29, 145, 192), (34, 94, 168), (12, 44, 132)],
    'ylgnbu8': [(255, 255, 217), (237, 248, 177), (199, 233, 180), (127, 205, 187), (65, 182, 196), (29, 145, 192), (34, 94, 168), (12, 44, 132)],
    'ylgnbu9': [(255, 255, 217), (237, 248, 177), (199, 233, 180), (127, 205, 187), (65, 182, 196), (29, 145, 192), (34, 94, 168), (37, 52, 148), (8, 29, 88)],
    'ylorbr3': [(255, 247, 188), (254, 196, 79), (217, 95, 14)],
    'ylorbr4': [(255, 255, 212), (254, 217, 142), (254, 153, 41), (204, 76, 2)],
    'ylorbr5': [(255, 255, 212), (254, 217, 142), (254, 153, 41), (217, 95, 14), (153, 52, 4)],
    'ylorbr6': [(255, 255, 212), (254, 227, 145), (254, 196, 79), (254, 153, 41), (217, 95, 14), (153, 52, 4)],
    'ylorbr7': [(255, 255, 212), (254, 227, 145), (254, 196, 79), (254, 153, 41), (236, 112, 20), (204, 76, 2), (140, 45, 4)],
    'ylorbr8': [(255, 255, 229), (255, 247, 188), (254, 227, 145), (254, 196, 79), (254, 153, 41), (236, 112, 20), (204, 76, 2), (140, 45, 4)],
    'ylorbr9': [(255, 255, 229), (255, 247, 188), (254, 227, 145), (254, 196, 79), (254, 153, 41), (236, 112, 20), (204, 76, 2), (153, 52, 4), (102, 37, 6)],
    'ylorrd3': [(255, 237, 160), (254, 178, 76), (240, 59, 32)],
    'ylorrd4': [(255, 255, 178), (254, 204, 92), (253, 141, 60), (227, 26, 28)],
    'ylorrd5': [(255, 255, 178), (254, 204, 92), (253, 141, 60), (240, 59, 32), (189, 0, 38)],
    'ylorrd6': [(255, 255, 178), (254, 217, 118), (254, 178, 76), (253, 141, 60), (240, 59, 32), (189, 0, 38)],
    'ylorrd7': [(255, 255, 178), (254, 217, 118), (254, 178, 76), (253, 141, 60), (252, 78, 42), (227, 26, 28), (177, 0, 38)],
    'ylorrd8': [(255, 255, 204), (255, 237, 160), (254, 217, 118), (254, 178, 76), (253, 141, 60), (252, 78, 42), (227, 26, 28), (177, 0, 38)],
}


if __name__ == '__main__':
    main()
