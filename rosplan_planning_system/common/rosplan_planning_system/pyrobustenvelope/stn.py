import re
from fractions import Fraction

class TimePoint(object):
    def __init__(self, name, kind, action_name=None, parameters=None,
                 instance_name=None, til=None):
        self.name = name
        assert kind in ["zero", "til", "instantaneous", "start", "end"]
        self.kind = kind
        self.action_name = action_name
        self.parameters = parameters
        self.instance_name = instance_name
        self.til = til
        self.paired_time_point = None

    def is_durative_extreme(self):
        return self.kind in ["start", "end"]

    def is_start(self):
        return self.kind == "start"

    def is_end(self):
        return self.kind == "end"

    def is_til(self):
        return self.kind == "til"

    def is_instantaneous_action(self):
        return self.kind == "instantaneous"

    def __hash__(self):
        h = hash(self.name)
        h += hash(self.kind)
        h += hash(self.action_name)
        if self.parameters:
            for p in self.parameters:
                h += hash(p)
        h += hash(self.instance_name)
        h += hash(self.til)
        return h

    def __eq__(self, oth):
        return self.name == oth.name and \
        self.kind == oth.kind and \
        self.action_name == oth.action_name and \
        self.parameters == oth.parameters and \
        self.instance_name == oth.instance_name and \
        self.til == oth.til

class Constraint(object):
    def __init__(self, src, dst, lower_bound=None, upper_bound=None):
        self.src = src
        self.dst = dst
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def is_parametric(self):
        return isinstance(self.lower_bound, Parameter) or \
            isinstance(self.upper_bound, Parameter)

class Parameter(object):
    def __init__(self, name, weight=1, default=None, lower_bound=None, upper_bound=None):
        self.name = name
        self.weight = weight
        self.default = default
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def __str__(self):
        return self.name

    def __hash__(self):
        return hash(self.name)
    def __eq__(self, oth):
        return self.name == oth.name
    def __ne__(self, oth):
        return self.name != oth.name


class STN(object):

    m_parameter = re.compile(r'parameter\s+([A-Za-z_][A-Za-z0-9_]*)(\s+weight\s+((\d+)(\.\d+)?))?(\s+default\s+((\d+)(\.\d+)?))?(\s+lb\s+((\d+)(\.\d+)?))?(\s+ub\s+((\d+)(\.\d+)?))?\s*;')
	# SYNTAX: action_name: (param1, param2, param3)
    #m_action = re.compile(r'(durative-)?action\s+instance\s+([A-Za-z_][A-Za-z0-9_]*)\s*:\s*([A-Za-z_][A-Za-z0-9_]*)((\(\))|(\(([A-Za-z_][A-Za-z0-9_]*)(\s*,\s*[A-Za-z_][A-Za-z0-9_]*)*\)))\s*;')
	# SYNTAX: (action_name param1 param2 param3)
    m_action = re.compile(r'(durative-)?action\s+instance\s+([A-Za-z_][A-Za-z0-9_]*)\s*:\s*\(([A-Za-z_][A-Za-z0-9_]*)((\s+([A-Za-z_][A-Za-z0-9_]*))*)\)\s*;')
    m_constraint = re.compile(r'c\s*:\s*((\.zero)|(([A-Za-z_][A-Za-z0-9_]*)(\.((start)|(end)))?))\s*-\s*((\.zero)|(([A-Za-z_][A-Za-z0-9_]*)(\.((start)|(end)))?))\s+in\s*\[\s*(((\+|-)?(inf)|((\d+)(\.\d+)?)|((\d+)/(\d+)))|([A-Za-z_][A-Za-z0-9_]*)),\s*(((\+|-)?(inf)|((\d+)(\.\d+)?)|((\d+)/(\d+)))|([A-Za-z_][A-Za-z0-9_]*))\s*]\s*;')

    def __init__(self, fname=None):
        self.time_points = {}
        self.constraints = []
        self.parameters = {}
        self.zero = None

        if fname:
            self.read_from_file(fname)

    def is_parametric(self):
        return len(self.parameters) > 0

    def read_from_file(self, fname):
        self.zero = TimePoint(".zero", "zero")
        self.time_points[".zero"] = self.zero
        with open(fname) as fh:
            line_cnt = 0
            for line in fh:
                line_cnt += 1
                line = line.strip()
                if line.startswith('#') or len(line) == 0:
                    pass # This is a comment
                else:
                    m = STN.m_action.match(line)
                    if m:
                        params = []
                        if m.group(4):
                            sparams = m.group(4).strip()
                            if sparams == "":
                                params = []
                            else:
                                params = [x.strip() for x in sparams.split()]
                        if m.group(1) == "durative-":
                            st = TimePoint(m.group(2) + ".start",
                                           "start",
                                           action_name=m.group(3),
                                           parameters=params,
                                           instance_name=m.group(2))
                            self.time_points[st.name] = st
                            et = TimePoint(m.group(2) + ".end",
                                           "end",
                                           action_name=m.group(3),
                                           parameters=params,
                                           instance_name=m.group(2))
                            self.time_points[et.name] = et
                            st.paired_time_point = et
                            et.paired_time_point = st
                        else:
                            t = TimePoint(m.group(2),
                                          "instantaneous",
                                          action_name=m.group(3),
                                          parameters=params,
                                          instance_name=m.group(2))
                            self.time_points[t.name] = t
                        continue
                    m = STN.m_parameter.match(line)
                    if m:
                        name = m.group(1)
                        if name == 'inf':
                            raise SyntaxError('Parameter names cannot be "inf"'
                                              'at line %d' % line_cnt)
                        if name in self.parameters:
                            raise SyntaxError('Parameter "%s" already declared '
                                              'at line %d' % (name, line_cnt))
                        self.parameters[name] = Parameter(name)
                        if m.group(3):
                            self.parameters[name].weight = Fraction(m.group(3))
                        if m.group(7):
                            self.parameters[name].default = Fraction(m.group(7))
                        if m.group(11):
                            self.parameters[name].lower_bound = Fraction(m.group(11))
                        if m.group(15):
                            self.parameters[name].upper_bound = Fraction(m.group(15))
                        continue
                    m = STN.m_constraint.match(line)
                    if m:
                        d = m.group(1)
                        s = m.group(9)
                        l = m.group(17)
                        u = m.group(28)

                        try:
                            ts = self.time_points[s]
                        except KeyError:
                            raise SyntaxError('Time point %s undeclared '
                                            'at line %d' % (s, line_cnt))
                        try:
                            td = self.time_points[d]
                        except KeyError:
                            raise SyntaxError('Time point %s '
                                              'undeclared at line'
                                              ' %d' % (d, line_cnt))

                        if l == '-inf':
                            lb = None
                        elif l == '+inf' or l == 'inf':
                            raise SyntaxError('Invalid lower bound '
                                              'at line %d: %s' \
                                              % (line_cnt, l))
                        elif l in self.parameters:
                            lb = self.parameters[l]
                        else:
                            lb = Fraction(l)

                        if u == '-inf':
                            raise SyntaxError('Invalid upper bound '
                                              'at line %d: %s' \
                                              % (line_cnt, l))
                        elif u == 'inf' or u == '+inf':
                            ub = None
                        elif u in self.parameters:
                            ub = self.parameters[u]
                        else:
                            ub = Fraction(u)

                        c = Constraint(ts, td, lb, ub)
                        self.constraints.append(c)

                    else:
                        raise SyntaxError('Unable to parse STN plan '
                                          ' at line %d ("%s")' \
                                          % (line_cnt, line))
                    continue

    def __str__(self):
        res = []
        p = res.append
        for par in self.parameters.values():
            p('parameter %s weight %s%s;' % (par.name, par.weight, (' default %s' % par.default) if par.default is not None else ''))
        for t in self.time_points.values():
            if t != self.zero:
                if t.is_instantaneous_action():
                    p('action instance %s : (%s %s);' % (t.name,
                                                       t.action_name,
                                                       " ".join(t.parameters)))
                elif t.is_start():
                    p('durative-action instance %s : (%s %s);' % \
                      (t.instance_name, t.action_name, " ".join(t.parameters)))

        for c in self.constraints:
            p('c: %s - %s in [%s, %s];' % \
              (c.dst.name, c.src.name,
               c.lower_bound if c.lower_bound is not None else '-inf',
               c.upper_bound if c.upper_bound is not None else '+inf'))
        return '\n'.join(res)


    def to_dot(self):
        res = ["digraph {"]
        p = res.append
        for c in self.constraints:
            p('"%s" -> "%s" [label="[%s, %s]"];' % \
              (c.dst.name, c.src.name,
               float(c.lower_bound) if c.lower_bound is not None else '-inf',
               float(c.upper_bound) if c.upper_bound is not None else '+inf'))

        p("}")
        return '\n'.join(res)

