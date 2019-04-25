import warnings
from fractions import Fraction

from rosplan_planning_system.pypddlplus.model import *
from rosplan_planning_system.pypddlplus.parser import PddlPlusParser


class PddlPlusSemantics(object):

    def proposition(self, ast):
        if ast.const_bool:
            if ast.const_bool == 'true':
                return PddlBoolConstant(True)
            else:
                return PddlBoolConstant(False)
        else:
            return PddlApply(ast.name, ast.parameters)

    def function_name(self, ast):
        return PddlApply(ast.name, ast.parameters)

    def untimed_condition(self, ast):
        if ast.proposition:
            return ast.proposition
        else:
            if ast.op == 'not':
                assert len(ast.expressions) == 1
                return PddlNot(ast.expressions[0])
            elif ast.op == 'and':
                return PddlAnd(ast.expressions)
            elif ast.op == 'or':
                return PddlOr(ast.expressions)
            elif ast.op == 'imply':
                assert len(ast.expressions) == 2
                return PddlImplies(ast.expressions[0], ast.expressions[1])
            else:
                assert len(ast.expressions) == 2
                return PddlArithRelation(ast.op, ast.expressions[0], ast.expressions[1])

    def untimed_effect(self, ast):
        if ast.proposition:
            return ast.proposition
        elif ast.negated_proposition:
            return PddlNot(ast.negated_proposition)
        elif ast.kind:
            if ast.kind == 'increase':
                return PddlIncrease(ast.function, ast.expr)
            elif ast.kind == "decrease":
                return PddlDecrease(ast.function, ast.expr)
            elif ast.kind == "assign":
                return PddlAssign(ast.function, ast.expr)
            else:
                assert False
        else:
            assert False

    def float_expr(self, ast):
        if ast.number:
            return PddlFloatConstant(Fraction(ast.number))
        elif ast.function:
            return ast.function
        elif ast.param_name:
            return PddlSynthParam(name=ast.param_name)
        elif ast.duration:
            return PddlDurationVar()
        else:
            if ast.op == '-' and len(ast.expressions) == 1:
                return PddlArithOperator("*", [ast.expressions[0],
                                               PddlFloatConstant(-1)])
            else:
                return PddlArithOperator(ast.op, ast.expressions)

    def float_expr_t(self, ast):
        if ast.number:
            return PddlFloatConstant(Fraction(ast.number))
        elif ast.time_var:
            return PddlTimeVar()
        elif ast.param_name:
            return PddlSynthParam(name=ast.param_name)
        elif ast.function:
            return ast.function
        elif ast.duration:
            return PddlDurationVar()
        else:
            if ast.op == '-' and len(ast.expressions) == 1:
                return PddlArithOperator("*", [ast.expressions[0],
                                               PddlFloatConstant(-1)])
            else:
                return PddlArithOperator(ast.op, ast.expressions)

    def timed_condition(self, ast):
        if ast.op == 'and':
            return PddlAnd(ast.expressions)
        else:
            if ast.time_ref == 'all':
                return PddlOverAll(ast.condition)
            elif ast.time_ref == 'start':
                return PddlAtStart(ast.condition)
            elif ast.time_ref == 'end':
                return PddlAtEnd(ast.condition)
            else:
                assert False, ast.time_ref

    def timed_effect(self, ast):
        if ast.kind == "start":
            return PddlAtStart(ast.effect)
        elif ast.kind == "end":
            return PddlAtEnd(ast.effect)
        elif ast.kind == "increase":
            return PddlIncrease(ast.function, ast.expr)
        elif ast.kind == "decrease":
            return PddlDecrease(ast.function, ast.expr)
        else:
            assert False

    def process_effect(self, ast):
        if ast.kind == "increase":
            return PddlIncrease(ast.function, ast.expr)
        elif ast.kind == "decrease":
            return PddlDecrease(ast.function, ast.expr)
        else:
            assert False

    def duration_constraint(self, ast):
        if ast.op == 'and':
            return [x for y in ast.expressions for x in y]
        else:
            return [PddlArithRelation(ast.op, PddlDurationVar(), ast.expr)]

    def domain(self, ast):
        res = PddlDomain(name=ast.name)

        # requirements
        reqs = [x for x in ast.body.preamble if x.requirements]
        assert len(reqs) == 1
        res.requirements = set(reqs[0].requirements)

        #types
        ts = [x for x in ast.body.preamble if x.types]
        if ts:
            assert len(ts) == 1
            for x in ts[0].types:
                parents = [res.types[p] for p in x.parents if p in res.types]
                for n in x.names:
                    t = PddlType(n, parents)
                    res.types[n] = t

        #predicates
        ps = [x for x in ast.body.preamble if x.predicates]
        if ps:
            assert len(ps) == 1
            for x in ps[0].predicates:
                pars = []
                for tv in x.formal_parameters:
                    types = [res.types[t] for t in tv.types]
                    if len(types) == 1:
                        for y in tv.var_names:
                            pars.append(PddlFormalParameter(y, types[0]))
                    else:
                        # we create a dummy either type just for this parameter
                        t = PddlType(res.fresh_type_name(), types)
                        for y in tv.var_names:
                            pars.append(PddlFormalParameter(y, t))
                res.predicates[x.name] = PddlPredicate(x.name, pars)

        #functions
        ps = [x for x in ast.body.preamble if x.functions]
        if ps:
            assert len(ps) == 1
            for x in ps[0].functions:
                pars = []
                for tv in x.formal_parameters:
                    types = [res.types[t] for t in tv.types]
                    if len(types) == 1:
                        for y in tv.var_names:
                            pars.append(PddlFormalParameter(y, types[0]))
                    else:
                        # we create a dummy either type just for this parameter
                        t = PddlType(res.fresh_type_name(), types)
                        for y in tv.var_names:
                            pars.append(PddlFormalParameter(y, t))
                res.functions[x.name] = PddlFunction(x.name, pars)

        #synth-params
        ps = [x for x in ast.body.preamble if x.synth_parameters]
        if ps:
            assert len(ps) == 1
            for x in ps[0].synth_parameters:
                res.synth_parameters.append(PddlSynthParam(x))

        # actions
        acts = [x.action for x in ast.body.structures if x.action]
        for x in acts:
            pars = []
            a = PddlAction(x.name)
            for tv in x.formal_parameters:
                types = [res.types[t] for t in tv.types]
                if len(types) == 1:
                    for y in tv.var_names:
                        pars.append((y, PddlActionParameter(y, types[0], a)))
                else:
                    # we create a dummy either type just for this parameter
                    t = PddlType(res.fresh_type_name(), types)
                    for y in tv.var_names:
                        pars.append((y, PddlActionParameter(y, t, a)))
            a.parameters = pars
            res.actions[a.name] = a
            a.precondition = x.precondition
            a.effect = x.effect
            a.synth_parameters = [PddlSynthParam(p) for p in x.synth_params] if x.synth_params else []

        # durative-actions
        acts = [x.durative_action for x in ast.body.structures if x.durative_action]
        for x in acts:
            pars = []
            a = PddlDurativeAction(x.name)
            for tv in x.formal_parameters:
                types = [res.types[t] for t in tv.types]
                if len(types) == 1:
                    for y in tv.var_names:
                        pars.append((y, PddlActionParameter(y, types[0], a)))
                else:
                    # we create a dummy either type just for this parameter
                    t = PddlType(res.fresh_type_name(), types)
                    for y in tv.var_names:
                        pars.append((y, PddlActionParameter(y, t, a)))
            a.parameters = pars
            res.durative_actions[a.name] = a
            a.duration_constraint = x.duration_constraint
            a.condition = x.condition
            a.effect = x.effect
            a.synth_parameters = [PddlSynthParam(p) for p in x.synth_params] if x.synth_params else []

        # events
        acts = [x.event for x in ast.body.structures if x.event]
        for x in acts:
            pars = []
            a = PddlEvent(x.name)
            for tv in x.formal_parameters:
                types = [res.types[t] for t in tv.types]
                if len(types) == 1:
                    for y in tv.var_names:
                        pars.append((y, PddlActionParameter(y, types[0], a)))
                else:
                    # we create a dummy either type just for this parameter
                    t = PddlType(res.fresh_type_name(), types)
                    for y in tv.var_names:
                        pars.append((y, PddlActionParameter(y, t, a)))
            a.parameters = pars
            res.events[x.name] = a
            a.precondition = x.precondition
            a.effect = x.effect

        # processes
        acts = [x.process for x in ast.body.structures if x.process]
        for x in acts:
            pars = []
            a = PddlProcess(x.name)
            for tv in x.formal_parameters:
                types = [res.types[t] for t in tv.types]
                if len(types) == 1:
                    for y in tv.var_names:
                        pars.append((y, PddlActionParameter(y, types[0], a)))
                else:
                    # we create a dummy either type just for this parameter
                    t = PddlType(res.fresh_type_name(), types)
                    for y in tv.var_names:
                        pars.append((y, PddlActionParameter(y, t, a)))
            a.parameters = pars
            res.processes[x.name] = a
            a.precondition = x.precondition
            a.effect = x.effect

        return res

    def init_element(self, ast):
        if ast.op == '=':
            return PddlArithRelation('=', ast.function,
                                     PddlFloatConstant(Fraction(ast.value)))
        elif ast.op == 'not':
            return PddlNot(PddlApply(ast.name, ast.parameters))
        elif ast.op == 'at':
            return PddlAtTime(Fraction(ast.time), ast.element)
        elif ast.name:
            return PddlApply(ast.name, ast.parameters)
        else:
            print(ast)
            assert False

    def problem(self, ast):
        res = PddlProblem(name=ast.name)

        # requirements
        reqs = [x.requirements for x in ast.body if x.requirements]
        if reqs:
            assert len(reqs) == 1
            res.requirements = set(reqs[0])

        # domain_name
        doms = [x.domain_name for x in ast.body if x.domain_name]
        if doms:
            assert len(doms) == 1
            res.domain_name = doms[0]

        # objects
        objs = [x.objects for x in ast.body if x.objects]
        if objs:
            assert len(objs) == 1
            for x in objs[0]:
                if len(x.types) == 1:
                    for on in x.obj_names:
                        res.objects[on] = PddlObject(on, x.types[0])
                else:
                    raise SyntaxError("Either type in objects still unsupported")

        # initial state
        inits = [x.init_list for x in ast.body if x.init_list]
        if inits:
            assert len(inits) == 1
            res.initial_state = inits[0]

        # goal
        goals = [x.goal for x in ast.body if x.goal]
        if goals:
            assert len(goals) == 1
            res.goal = goals[0]

        metrics = [x for x in ast.body if x.metric]
        if metrics:
            warnings.warn("Metrics are currently not supported")
        return res


def parse_domain_file(fname):
    with open(fname) as fh:
        parser = PddlPlusParser(semantics=PddlPlusSemantics())
        return parser.parse(fh.read(), filename=fname,
                            rule_name='domain_start')

def parse_problem_file(fname):
    with open(fname) as fh:
        parser = PddlPlusParser(semantics=PddlPlusSemantics())
        return parser.parse(fh.read(), filename=fname,
                            rule_name='problem_start')

def parse_instance(domain_file, problem_file):
    domain = parse_domain_file(domain_file)
    problem = parse_problem_file(problem_file)
    return PddlInstance(domain, problem)

