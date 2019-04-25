from six import PY2

def is_string(s):
    if PY2:
        if isinstance(s, unicode):
            return True
    return isinstance(s, str)

class PddlType(object):
    def __init__(self, name, parents):
        assert name
        self.name = name
        self.parents = list(parents)

    def __str__(self):
        if len(self.parents) == 0:
            return self.name
        elif len(self.parents) == 1:
            return self.name + " - " + str(self.parents[0])
        else:
            return self.name + " - (either " + \
                " ".join(str(x) for x in self.parents) + ")"
    __repr__ = __str__


class PddlFormalParameter(object):
    def __init__(self, name, type_):
        assert name
        self.name = name
        self.type = type_

    def __str__(self):
        return "%s - %s" % (self.name, self.type)
    __repr__ = __str__


class PddlActionParameter(object):
    def __init__(self, name, type_, action):
        assert name
        name = name.lower()
        self.name = name
        self.type = type_
        self.action = action

    def __str__(self):
        return "%s - %s" % (self.name, self.type)
    __repr__ = __str__

class PddlPredicate(object):
    def __init__(self, name, formal_parameters=None):
        self.name = name
        self.formal_parameters = list(formal_parameters) if formal_parameters else []

    def __str__(self):
        if self.formal_parameters:
            return "(" + str(self.name) + " " + \
                " ".join(str(x) for x in self.formal_parameters) + ")"
        else:
            return "(%s)" % self.name
    __repr__ = __str__

class PddlFunction(object):
    def __init__(self, name, formal_parameters=None):
        self.name = name
        self.formal_parameters = list(formal_parameters) if formal_parameters else []

    def __str__(self):
        if self.formal_parameters:
            return "(" + str(self.name) + " " + \
                " ".join(str(x) for x in self.formal_parameters) + ")"
        else:
            return "(%s)" % self.name
    __repr__ = __str__

class PddlExpression(object):
    def resove_apply(self, params):
        raise NotImplementedError

    def ground_apply(self, par2obj, objects2ground):
        raise NotImplementedError

class PddlSynthParam(PddlExpression):
    def __init__(self, name):
        assert name
        self.name = name

    def resove_apply(self, params):
        return

    def ground_apply(self, par2obj, objects2ground):
        return PddlSynthParam(self.name)

    def __hash__(self):
        return hash(self.name)
    def __eq__(self, oth):
        return self.name == oth.name
    def __ne__(self, oth):
        return self.name != oth.name

    def __str__(self):
        return "%s" % (self.name)
    __repr__ = __str__


class PddlApply(PddlExpression):
    def __init__(self, function, actual_parameters=None):
        assert function
        self.function = function
        self.actual_parameters = actual_parameters if actual_parameters else []

    def resove_apply(self, params):
        if is_string(self.function):
            try:
                self.function = params[self.function]
            except KeyError:
                raise SyntaxError("Unable to resolve predicate/function name "
                                  "'%s' in '%s'" % (self.function, self))
        np = []
        for x in self.actual_parameters:
            if is_string(x):
                try:
                    np.append(params[x])
                except KeyError:
                    raise SyntaxError("Unable to resolve parameter "
                                      "'%s' in '%s'" % (x, self))
            else:
                np.append(x)
        self.actual_parameters = np

        # sanity checks
        if len(self.function.formal_parameters) != len(np):
            raise SyntaxError("Paramenter count mismatch in '%s'" % self)
        for i,fp in enumerate(self.function.formal_parameters):
            if fp.type != np[i].type and all(fp.type != x for x in np[i].type.parents):
                raise TypeError("Type error in parameter '%s' of '%s'" % (np[i], self))

    def ground_apply(self, par2obj, objects2ground):
        if self.actual_parameters:
            objs = []
            for x in self.actual_parameters:
                if x in par2obj:
                    objs.append(par2obj[x])
                else:
                    objs.append(x)
            key = (self.function, tuple(objs))
            return PddlApply(objects2ground[key])
        else:
            return self

    def __str__(self):
        if self.actual_parameters:
            ps = []
            for p in self.actual_parameters:
                if is_string(p):
                    ps.append("[%s]" % p)
                else:
                    ps.append(p.name)
            fn = self.function if is_string(self.function) \
                 else self.function.name
            return "(%s %s)" % (fn, " ".join(ps))
        else:
            if is_string(self.function):
                return "([%s])" % self.function
            else:
                return "(%s)" % self.function.name
    __repr__ = __str__

class PddlNot(PddlExpression):
    def __init__(self, expression):
        self.expression = expression

    def resove_apply(self, params):
        self.expression.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        return PddlNot(self.expression.ground_apply(par2obj, objects2ground))

    def __str__(self):
        return "(not %s)" % self.expression
    __repr__ = __str__

class PddlAnd(PddlExpression):
    def __init__(self, expressions):
        self.expressions = list(expressions)

    def resove_apply(self, params):
        for x in self.expressions:
            x.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        ne = [x.ground_apply(par2obj, objects2ground) for x in self.expressions]
        return PddlAnd(ne)

    def __str__(self):
        return "(and " + " ".join(str(x) for x in self.expressions) + ")"
    __repr__ = __str__

class PddlOr(PddlExpression):
    def __init__(self, expressions):
        self.expressions = list(expressions)

    def resove_apply(self, params):
        for x in self.expressions:
            x.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        ne = [x.ground_apply(par2obj, objects2ground) for x in self.expressions]
        return PddlOr(ne)

    def __str__(self):
        return "(or " + " ".join(str(x) for x in self.expressions) + ")"
    __repr__ = __str__

class PddlImplies(PddlExpression):
    def __init__(self, left, right):
        self.left = left
        self.right = right

    def resove_apply(self, params):
        self.left.resove_apply(params)
        self.right.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        l = self.left.ground_apply(par2obj, objects2ground)
        r = self.right.ground_apply(par2obj, objects2ground)
        return PddlImplies(l, r)

    def __str__(self):
        return "(implies %s %s)" % (self.left, self.right)
    __repr__ = __str__

class PddlBoolConstant(PddlExpression):
    def __init__(self, value):
        self.value = value

    def resove_apply(self, params):
        pass

    def ground_apply(self, par2obj, objects2ground):
        return self

    def __str__(self):
        return "true" if self.value else "false"
    __repr__ = __str__

class PddlArithRelation(PddlExpression):
    def __init__(self, op, left, right):
        assert op in ['>', '<', '>=', '<=', '=']
        self.op = op
        self.left = left
        self.right = right

    def resove_apply(self, params):
        self.left.resove_apply(params)
        self.right.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        l = self.left.ground_apply(par2obj, objects2ground)
        r = self.right.ground_apply(par2obj, objects2ground)
        return PddlArithRelation(self.op, l, r)

    def __str__(self):
        return "(%s %s %s)" % (self.op, self.left, self.right)
    __repr__ = __str__


class PddlArithOperator(PddlExpression):
    def __init__(self, op, operands):
        assert op in ['+', '-', '*', '/']
        self.op = op
        self.operands = list(operands)

    def resove_apply(self, params):
        for x in self.operands:
            x.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        no = [x.ground_apply(par2obj, objects2ground) for x in self.operands]
        return PddlArithOperator(self.op, no)

    def __str__(self):
        return ("(%s " % self.op) + " ".join(str(x) for x in self.operands) + ")"
    __repr__ = __str__

class PddlFloatConstant(PddlExpression):
    def __init__(self, value):
        self.value = value

    def resove_apply(self, params):
        pass

    def ground_apply(self, par2obj, objects2ground):
        return self

    def __str__(self):
        return "%s" % self.value
    __repr__ = __str__

class PddlOverAll(PddlExpression):
    def __init__(self, expression):
        self.expression = expression

    def resove_apply(self, params):
        self.expression.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        return PddlOverAll(self.expression.ground_apply(par2obj, objects2ground))

    def __str__(self):
        return "(over all %s)" % self.expression
    __repr__ = __str__


class PddlAtStart(PddlExpression):
    def __init__(self, expression):
        self.expression = expression

    def resove_apply(self, params):
        self.expression.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        return PddlAtStart(self.expression.ground_apply(par2obj, objects2ground))

    def __str__(self):
        return "(at start %s)" % self.expression
    __repr__ = __str__


class PddlAtEnd(PddlExpression):
    def __init__(self, expression):
        self.expression = expression

    def resove_apply(self, params):
        self.expression.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        return PddlAtEnd(self.expression.ground_apply(par2obj, objects2ground))

    def __str__(self):
        return "(at end %s)" % self.expression
    __repr__ = __str__

class PddlAtTime(PddlExpression):
    def __init__(self, time, expression):
        assert time
        assert expression
        self.time = time
        self.expression = expression

    def resove_apply(self, params):
        self.expression.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        return PddlAtTime(self.time,
                          self.expression.ground_apply(par2obj, objects2ground))

    def __str__(self):
        return "(at %s %s)" % (self.time, self.expression)
    __repr__ = __str__


class PddlIncrease(PddlExpression):
    def __init__(self, function, expression):
        self.function = function
        self.expression = expression

    def resove_apply(self, params):
        self.function.resove_apply(params)
        self.expression.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        f = self.function.ground_apply(par2obj, objects2ground)
        e = self.expression.ground_apply(par2obj, objects2ground)
        return PddlIncrease(f, e)

    def __str__(self):
        return "(increase %s %s)" % (self.function, self.expression)
    __repr__ = __str__

class PddlDecrease(PddlExpression):
    def __init__(self, function, expression):
        self.function = function
        self.expression = expression

    def resove_apply(self, params):
        self.function.resove_apply(params)
        self.expression.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        f = self.function.ground_apply(par2obj, objects2ground)
        e = self.expression.ground_apply(par2obj, objects2ground)
        return PddlDecrease(f, e)

    def __str__(self):
        return "(decrease %s %s)" % (self.function, self.expression)
    __repr__ = __str__


class PddlAssign(PddlExpression):
    def __init__(self, function, expression):
        self.function = function
        self.expression = expression

    def resove_apply(self, params):
        self.function.resove_apply(params)
        self.expression.resove_apply(params)

    def ground_apply(self, par2obj, objects2ground):
        f = self.function.ground_apply(par2obj, objects2ground)
        e = self.expression.ground_apply(par2obj, objects2ground)
        return PddlAssign(f, e)

    def __str__(self):
        return "(assign %s %s)" % (self.function, self.expression)
    __repr__ = __str__


class PddlTimeVar(PddlExpression):
    def resove_apply(self, params):
        pass

    def ground_apply(self, par2obj, objects2ground):
        return self

    def __str__(self):
        return "#t"
    __repr__ = __str__


class PddlDurationVar(PddlExpression):
    def resove_apply(self, params):
        pass

    def ground_apply(self, par2obj, objects2ground):
        return self

    def __str__(self):
        return "?duration"
    __repr__ = __str__


class PddlAction(object):
    def __init__(self, name, parameters=None):
        self.name = name
        self.parameters = list(parameters) if parameters else []
        self.precondition = None
        self.effect = []
        self.synth_parameters = []

    def __str__(self):
        return """
    PddlAction(%s):
        parameters:%s
        precondition:%s
        effect:%s
        synth-parameters:%s
               """ % (self.name, self.parameters,
                      self.precondition, self.effect,
                      self.synth_parameters)
    __repr__ = __str__

class PddlDurativeAction(object):
    def __init__(self, name, parameters=None):
        name = name.lower()
        self.name = name
        self.parameters = list(parameters) if parameters else []
        self.duration_constraint = []
        self.condition = None
        self.effect = []
        self.synth_parameters = []

    def __str__(self):
        return """
    PddlDurativeAction(%s):
        parameters:%s
        duration-constraint:%s
        condition:%s
        effect:%s
        synth-parameters:%s
               """ % (self.name, self.parameters,
                      self.duration_constraint,
                      self.condition, self.effect,
                      self.synth_parameters)
    __repr__ = __str__

class PddlProcess(object):
    def __init__(self, name, parameters=None):
        self.name = name
        self.parameters = list(parameters) if parameters else []
        self.precondition = None
        self.effect = []

    def __str__(self):
        return """
    PddlProcess(%s):
        parameters:%s
        precondition:%s
        effect:%s
               """ % (self.name, self.parameters,
                      self.precondition, self.effect)
    __repr__ = __str__

class PddlEvent(object):
    def __init__(self, name, parameters=None):
        self.name = name
        self.parameters = list(parameters) if parameters else []
        self.precondition = None
        self.effect = []

    def __str__(self):
        return """
    PddlEvent(%s):
        parameters:%s
        precondition:%s
        effect:%s
               """ % (self.name, self.parameters,
                      self.precondition, self.effect)
    __repr__ = __str__


class PddlDomain(object):
    def __init__(self, name):
        self.name = name
        self.requirements = set()
        self.types = {}
        self.predicates = {}
        self.functions = {}
        self.actions = {}
        self.durative_actions = {}
        self.processes = {}
        self.events = {}
        self.synth_parameters = []

        self._name_counter = 0

    def fresh_type_name(self, tplate = "either_type_%d"):
        while tplate % self._name_counter in self.types:
            self._name_counter += 1
        res = tplate % self._name_counter
        self._name_counter += 1
        return res

    def __str__(self):
        return """
PddlDomain(%s):
    requirements:%s
    types:%s
    predicates:%s
    functions:%s
    synth-parameters:%s
    actions:%s
    durative-actions:%s
    processes:%s
    events:%s
               """ % (self.name, self.requirements,
                      self.types, self.predicates, self.functions,
                      self.synth_parameters, self.actions, self.durative_actions,
                      self.processes, self.events)

class PddlProblem(object):
    def __init__(self, name):
        self.name = name
        self.domain_name = None
        self.requirements = set()
        self.objects = {}
        self.initial_state = []
        self.goal = None
        self.metric = None

    def __str__(self):
        return """
PddlProblem(%s):
    domain_name: %s
    requirements: %s
    objects: %s
    initial_state: %s
    goal: %s
    metric: %s
                """ % (self.name, self.domain_name, self.requirements,
                      self.objects, self.initial_state, self.goal,
                      self.metric)

class PddlObject(object):
    def __init__(self, name, type_):
        assert type_
        assert name
        self.name = name
        self.type = type_

    def __str__(self):
        return self.name + " - " + str(self.type)
    __repr__ = __str__


class PddlInstance(object):
    def __init__(self, domain, problem, mapback=None):
        self.domain = domain
        self.problem = problem
        self.mapback = mapback

        if problem.domain_name:
            if problem.domain_name != domain.name:
                raise SyntaxError("Domain name in problem and domain files mismatch!")

        #merging the data
        self.requirements = set(self.domain.requirements | self.problem.requirements)
        self.domain.requirements = self.requirements
        self.problem.requirements = self.requirements

        base = dict(self.domain.predicates)
        base.update(self.domain.functions)

        for a in self.domain.actions.values():
            pars = {}
            pars.update(base)
            pars.update(dict(a.parameters))
            a.precondition.resove_apply(pars)
            for e in a.effect:
                e.resove_apply(pars)

        for a in self.domain.durative_actions.values():
            pars = {}
            pars.update(base)
            pars.update(dict(a.parameters))
            a.condition.resove_apply(pars)
            for e in a.effect:
                e.resove_apply(pars)
            for d in a.duration_constraint:
                d.resove_apply(pars)

        for a in self.domain.events.values():
            pars = {}
            pars.update(base)
            pars.update(dict(a.parameters))
            a.precondition.resove_apply(pars)
            for e in a.effect:
                e.resove_apply(pars)

        for a in self.domain.processes.values():
            pars = {}
            pars.update(base)
            pars.update(dict(a.parameters))
            a.precondition.resove_apply(pars)
            for e in a.effect:
                e.resove_apply(pars)

        for o in self.problem.objects.values():
            o.type = self.domain.types[o.type]

        pars = {}
        pars.update(base)
        pars.update(self.problem.objects)
        for e in self.problem.initial_state:
            e.resove_apply(pars)

        if self.problem.goal:
            self.problem.goal.resove_apply(pars)


    def all_combinations(self, pars, type2objects):
        if len(pars) == 0:
            return [[]]
        else:
            p = pars[0]
            res = []
            candidates = list(type2objects[p.type])
            for t in type2objects:
                if p.type in t.parents:
                    candidates += list(type2objects[t])
            for o in candidates:
                for lst in self.all_combinations(pars[1:], type2objects):
                    res.append([o] + lst)
            return res

    def ground(self):
        # Build type 2 object map
        type2objects = {t : [] for t in self.domain.types.values()}
        for o in self.problem.objects.values():
            type2objects[o.type].append(o)

        dom = PddlDomain(self.domain.name + "_ground")
        prob = PddlProblem(self.problem.name + "_ground")
        prob.domain_name = dom.name

        dom.requirements = self.requirements
        prob.requirements = self.requirements

        dom.types = self.domain.types

        dom.synth_parameters = self.domain.synth_parameters

        name2objects = {}
        objects2ground = {}

        # Grounding predicates
        for p in self.domain.predicates.values():
            if p.formal_parameters:
                for fps in self.all_combinations(p.formal_parameters, type2objects):
                    n = p.name + "_" + "_".join(x.name for x in fps)
                    name2objects[n] = (p, fps)
                    r = PddlPredicate(n)
                    objects2ground[(p, tuple(fps))] = r
                    dom.predicates[n] = r
            else:
                dom.predicates[p.name] = p

        # Grounding functions
        for p in self.domain.functions.values():
            if p.formal_parameters:
                for fps in self.all_combinations(p.formal_parameters, type2objects):
                    n = p.name + "_" + "_".join(x.name for x in fps)
                    name2objects[n] = (p, fps)
                    r = PddlFunction(n)
                    objects2ground[(p, tuple(fps))] = r
                    dom.functions[n] = r
            else:
                dom.functions[p.name] = p

        #Grounding actions
        for a in self.domain.actions.values():
            if a.parameters:
                for fps in self.all_combinations([x[1] for x in a.parameters], type2objects):
                    n = a.name + "_" + "_".join(x.name for x in fps)
                    name2objects[n] = (a, fps)
                    na = PddlAction(n)
                    par2obj = {fp[1] : fps[i] for i,fp in enumerate(a.parameters)}
                    na.precondition = a.precondition.ground_apply(par2obj, objects2ground)
                    na.effect = [e.ground_apply(par2obj, objects2ground) for e in a.effect]
                    na.synth_parameters = a.synth_parameters
                    dom.actions[n] = na
            else:
                dom.actions[a.name] = a

        for a in self.domain.durative_actions.values():
            if a.parameters:
                for fps in self.all_combinations([x[1] for x in a.parameters], type2objects):
                    n = a.name + "_" + "_".join(x.name for x in fps)
                    name2objects[n] = (a, fps)
                    na = PddlDurativeAction(n)
                    par2obj = {fp[1] : fps[i] for i,fp in enumerate(a.parameters)}
                    na.condition = a.condition.ground_apply(par2obj, objects2ground)
                    na.effect = [e.ground_apply(par2obj, objects2ground) for e in a.effect]
                    na.duration_constraint = [e.ground_apply(par2obj, objects2ground)
                                              for e in a.duration_constraint]
                    na.synth_parameters = a.synth_parameters
                    dom.durative_actions[n] = na
            else:
                dom.durative_actions[a.name] = a

        for a in self.domain.events.values():
            if a.parameters:
                for fps in self.all_combinations([x[1] for x in a.parameters], type2objects):
                    n = a.name + "_" + "_".join(x.name for x in fps)
                    name2objects[n] = (a, fps)
                    na = PddlEvent(n)
                    par2obj = {fp[1] : fps[i] for i,fp in enumerate(a.parameters)}
                    na.precondition = a.precondition.ground_apply(par2obj, objects2ground)
                    na.effect = [e.ground_apply(par2obj, objects2ground) for e in a.effect]
                    dom.events[n] = na
            else:
                dom.events[a.name] = a

        for a in self.domain.processes.values():
            if a.parameters:
                for fps in self.all_combinations([x[1] for x in a.parameters], type2objects):
                    n = a.name + "_" + "_".join(x.name for x in fps)
                    name2objects[n] = (a, fps)
                    na = PddlProcess(n)
                    par2obj = {fp[1] : fps[i] for i,fp in enumerate(a.parameters)}
                    na.precondition = a.precondition.ground_apply(par2obj, objects2ground)
                    na.effect = [e.ground_apply(par2obj, objects2ground) for e in a.effect]
                    dom.processes[n] = na
            else:
                dom.processes[a.name] = a

        #grounding initial_state and goal
        prob.initial_state = [x.ground_apply({}, objects2ground)
                              for x in self.problem.initial_state]
        prob.goal = self.problem.goal.ground_apply({}, objects2ground)

        return PddlInstance(dom, prob, mapback=name2objects)

    def __str__(self):
        return "%s\n%s" % (self.domain, self.problem)
    __repr__ = __str__

